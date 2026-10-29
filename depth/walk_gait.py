#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class QuadCrawl(Node):

    def __init__(self):
        super().__init__('quad_crawl')

        # Robot geometry — same values as walk_simple.py
        self.L1 = 0.050
        self.L2 = 0.135
        self.L3 = 0.1575

        # self.nominal_z = 0.15
        # self.nominal_y_back = 0.010
        # self.nominal_y_front = 0.010

        # # Conservative initial crawl parameters
        # self.step_length = 0.030
        # self.step_height = 0.030

        # Gait parameters
        self.step_length = 0.030

        self.step_height_front = 0.030
        self.step_height_back = 0.040

        self.cycle_steps = 30
        self.timer_period = 0.05

        # Joint order: BR, BL, FL, FR
        self.joint_names = [
            'CCUP1', 'THIGH1', 'FOOT1',
            'CCUP2', 'THIGH2', 'FOOT2',
            'CCUP3', 'THIGH3', 'FOOT3',
            'CCUP4', 'THIGH4', 'FOOT4'
        ]

        # Crawl order: move one leg at a time.
        # Leg indices: 0=BR, 1=BL, 2=FL, 3=FR.
        self.phase_offsets = [0.0, 0.50, 0.0, 0.5]

        # Use the same controller topic as walk_simple.py.
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_controller/commands',
            10
        )

        # Standing joint angles must match your robot's known stable stance.
        # These are placeholders until we verify your launch stance values.
        # Standing angles observed in Gazebo
        # Joint order: BR, BL, FL, FR
        self.STAND = [
            0.0, -1.4002, -0.46137,  # BR
            0.0,  1.4002, -0.46137,  # BL
            0.0,  1.4002, -0.46137,  # FL
            0.0, -1.4002, -0.46137   # FR
        ]

        self.step = 0
        self.walking = False
        self.last_command = list(self.STAND)

        self.timer = self.create_timer(
            self.timer_period, self.update
        )

        threading.Thread(
            target=self.keyboard_input, daemon=True
        ).start()

        self.get_logger().info(
            "Quadruped crawl ready. Enter f to crawl, s to stop."
        )

    @staticmethod
    def clamp(value):
        return max(-1.0, min(1.0, value))

    def IK_left(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        J3 = math.acos(self.clamp(
            (L2**2 + L3**2 - G**2) / (2 * L2 * L3)
        )) - math.pi / 2

        J2 = math.atan2(-y, D) + math.acos(self.clamp(
            (L2**2 + G**2 - L3**2) / (2 * L2 * G)
        ))

        J1 = math.pi / 2 - math.atan2(-x, z) - math.atan2(D, L1)
        return J1, J2, J3

    def IK_right(self, x, y, z):
        L1, L2, L3 = self.L1, self.L2, self.L3

        C = math.sqrt(x**2 + z**2)
        D = math.sqrt(max(C**2 - L1**2, 0.0))
        G = math.sqrt(D**2 + y**2)

        J3 = math.acos(self.clamp(
            (L2**2 + L3**2 - G**2) / (2 * L2 * L3)
        )) - math.pi / 2

        J2 = -(math.atan2(-y, D) + math.acos(self.clamp(
            (L2**2 + G**2 - L3**2) / (2 * L2 * G)
        )))

        J1 = math.atan2(x, z) + math.atan2(D, L1) - math.pi / 2
        return J1, J2, J3

    @staticmethod
    def smoothstep(u):
        return 3.0 * u**2 - 2.0 * u**3

    def is_front(self, leg):
        return leg in (2, 3)

    def is_right(self, leg):
        return leg in (0, 3)

    # def foot_target(self, leg, phase):
    #     x = 0.05 if self.is_right(leg) else -0.05
    #     y0 = (
    #         self.nominal_y_front
    #         if self.is_front(leg)
    #         else self.nominal_y_back
    #     )
    #     z0 = self.nominal_z
    #     length = self.step_length

    #     if phase < 0.5:
    #         # Swing phase: lift the foot while moving forward.
    #         u = phase / 0.5
    #         s = self.smoothstep(u)
    #         y = y0 - length + 2.0 * length * s
    #         z = z0 - self.step_height * math.sin(math.pi * u)
    #     else:
    #         # Stance phase: return the foot along the ground.
    #         u = (phase - 0.5) / 0.5
    #         s = self.smoothstep(u)
    #         y = y0 + length - 2.0 * length * s
    #         z = z0

    #     return x, y, z

    def foot_target(self, leg, phase):
        x = 0.05 if self.is_right(leg) else -0.05

        height = (
            self.step_height_front
            if self.is_front(leg)
            else self.step_height_back
        )

        length = self.step_length

        # Fixed centre of the foot trajectory
        y0 = -0.010 if not self.is_front(leg) else 0.010
        z0 = 0.15

        # Centre -> left endpoint -> top -> right endpoint -> centre
        points = [
            (0.0, 0.0),
            (-length, 0.0),
            (0.0, -height),
            (length, 0.0),
            (0.0, 0.0),
        ]

        scaled = (phase % 1.0) * 4.0
        segment = min(int(scaled), 3)
        u = self.smoothstep(scaled - segment)

        y1, z1 = points[segment]
        y2, z2 = points[segment + 1]

        y = y0 + y1 + (y2 - y1) * u
        z = z0 + z1 + (z2 - z1) * u

        return x, y, z

    def ik_for_leg(self, leg, x, y, z):
        if self.is_right(leg):
            return self.IK_right(x, y, z)
        return self.IK_left(x, y, z)

    def calculate_stand_pose(self):
        stand = []

        for leg in range(4):
            x = 0.05 if leg in (0, 3) else -0.05
            y = (
                self.nominal_y_front
                if self.is_front(leg)
                else self.nominal_y_back
            )

            J1, J2, J3 = self.ik_for_leg(
                leg, x, y, self.nominal_z
            )
            stand.extend([J1, J2, J3])

        return stand

    
    def update(self):
        positions = []

        if self.walking:
            for leg in range(4):
                phase = (
                    self.step / self.cycle_steps
                    + self.phase_offsets[leg]
                ) % 1.0

                x, y, z = self.foot_target(leg, phase)
                positions.extend(self.ik_for_leg(leg, x, y, z))

            self.step = (self.step + 1) % self.cycle_steps

        else:
            # Hold the calculated standing configuration.
            positions = list(self.STAND)

        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)
        self.last_command = list(positions)


    def keyboard_input(self):
        while rclpy.ok():
            try:
                command = input("Command (f=crawl, s=stop): ").strip().lower()
            except EOFError:
                break

            if command == 'f':
                self.walking = True
                self.get_logger().info("Crawl started.")
            elif command == 's':
                self.walking = False
                self.get_logger().info("Crawl stopped; last joint command held.")
            else:
                print("Enter f or s.")

def main(args=None):
    rclpy.init(args=args)
    node = QuadCrawl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
