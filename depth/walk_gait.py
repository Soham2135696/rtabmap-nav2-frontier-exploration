#!/usr/bin/env python3

# import math
# import os
# import threading
# import xml.etree.ElementTree as ET

# import numpy as np
# import rclpy
# from ament_index_python.packages import get_package_share_directory
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray


# def rpy_matrix(roll, pitch, yaw):
#     cr, sr = math.cos(roll), math.sin(roll)
#     cp, sp = math.cos(pitch), math.sin(pitch)
#     cy, sy = math.cos(yaw), math.sin(yaw)
#     return np.array([
#         [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
#         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
#         [-sp, cp * sr, cp * cr],
#     ])


# def axis_rotation(axis, angle):
#     k = np.asarray(axis) / np.linalg.norm(axis)
#     K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
#     return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


# class Leg:
#     """One leg of quadruped.urdf: hip roll, hip pitch and knee joints, toe point.

#     Positions are in base_link: origin at the centre of the four hips,
#     x forward, y left, z up.
#     """

    # Toe: centre of the end of the foot mesh, in the foot link frame
    # (the foot mesh is mirrored between the left and right legs)
#     TOE = {'left': np.array([-0.0195, -0.2083, 0.0038]),
#            'right': np.array([0.0195, -0.2083, 0.0038])}

#     def __init__(self, joints):
        # joints: [(xyz, rotation, axis or None)] from base_link to the foot link
#         self.joints = joints
        # Hip: the first moving joint
#         p, R = np.zeros(3), np.eye(3)
#         for xyz, rot, axis in joints:
#             p = p + R @ xyz
#             R = R @ rot
#             if axis is not None:
#                 break
#         self.hip = p
#         self.toe = self.TOE['left' if p[1] > 0 else 'right']

#     def toe_position(self, q):
#         p, R, i = np.zeros(3), np.eye(3), 0
#         for xyz, rot, axis in self.joints:
#             p = p + R @ xyz
#             R = R @ rot
#             if axis is not None:
#                 R = R @ axis_rotation(axis, q[i])
#                 i += 1
#         return p + R @ self.toe

#     def ik(self, target, q):
#         """Joint angles that put the toe at target, starting from q (keeps that knee bend)."""
#         q = np.array(q, dtype=float)
#         for _ in range(20):
#             p = self.toe_position(q)
#             err = target - p
#             if np.linalg.norm(err) < 1e-5:
#                 break
#             J = np.zeros((3, 3))
#             for i in range(3):
#                 dq = q.copy()
#                 dq[i] += 1e-6
#                 J[:, i] = (self.toe_position(dq) - p) / 1e-6
#             q += np.linalg.solve(J.T @ J + 1e-6 * np.eye(3), J.T @ err)
#         return q


# def load_legs(urdf_path, feet):
#     root = ET.parse(urdf_path).getroot()
#     by_child = {j.find('child').get('link'): j for j in root.findall('joint')}

#     def origin(j):
#         o = j.find('origin')
#         xyz = np.array([float(v) for v in o.get('xyz', '0 0 0').split()])
#         rot = rpy_matrix(*[float(v) for v in o.get('rpy', '0 0 0').split()])
#         return xyz, rot

#     legs = []
#     for foot in feet:
#         chain, link = [], foot
#         while link != 'base_link':
#             j = by_child[link]
#             xyz, rot = origin(j)
#             axis = None
#             if j.get('type') != 'fixed':
#                 axis = [float(v) for v in j.find('axis').get('xyz').split()]
#             chain.append((xyz, rot, axis))
#             link = j.find('parent').get('link')
#         chain.reverse()
#         legs.append(Leg(chain))
#     return legs


# class QuadCrawl(Node):

#     def __init__(self):
#         super().__init__('quad_crawl')

#         p = self.declare_parameter
        # Gait parameters
#         self.step_length = p('step_length', 0.06).value      # m, body travel per cycle
#         self.step_height = p('step_height', 0.04).value      # m, toe lift in swing
#         self.cycle_time = p('cycle_time', 2.0).value         # s, all four legs step once
        # Body moves sideways away from the lifted legs, so the centre of mass
        # stays inside the triangle of the other three feet
#         self.sway = p('sway', 0.04).value                    # m
#         self.body_height = p('body_height', 0.20).value      # m, hip centre above the toes
#         self.stance_width = p('stance_width', 0.16).value    # m, toe distance from the centre line
        # Toes are centred under the centre of mass, 2.5 cm behind the hip centre
#         self.stance_x = p('stance_x', -0.025).value          # m
#         self.timer_period = 0.02

        # Joint order: BR, BL, FL, FR (same order as controllers.yaml)
#         self.joint_names = [
#             'base_link_Revolute_1', 'ccup_3_Revolute_31', 'thigh_3_Revolute_32',
#             'base_link_Revolute_3', 'ccup_2_Revolute_28', 'thigh_2_Revolute_29',
#             'base_link_Revolute_4', 'ccup_1_Revolute_25', 'thigh_1_Revolute_26',
#             'base_link_Revolute_2', 'ccup_4_Revolute_34', 'thigh_4_Revolute_35'
#         ]
        # Kinematics from the URDF; foot links in the same leg order
#         urdf = os.path.join(get_package_share_directory('depth'), 'urdf', 'quadruped.urdf')
#         self.legs = load_legs(urdf, ['foot_3', 'foot_2', 'foot_1', 'foot_4'])
        # Hip x of each leg (front legs +, back legs -)
#         self.hip_x = [leg.hip[0] for leg in self.legs]

        # Fastest a joint command may change while standing up or settling.
        # Jumping straight to a far pose (e.g. from the spawn pose) makes the
        # legs kick and flips the robot. The gait itself is not limited.
#         self.max_joint_speed = 1.0  # rad/s

        # Crawl, one leg at a time. Each cycle:
        #   0    - 0.25   all feet down: body forward half a step, sways right
        #   0.25 - 0.5    BL then FL step
        #   0.5  - 0.75   all feet down: body forward half a step, sways left
        #   0.75 - 1      BR then FR step
        # The body only moves while all four feet are down, so a foot that has
        # just landed (still lightly loaded) is not dragged.
        # Leg indices: 0=BR, 1=BL, 2=FL, 3=FR.
#         self.swing_start = [0.75, 0.25, 0.375, 0.875]
#         self.swing_time = 0.125
        # Toe x relative to the body over a cycle runs over one step length;
        # this offset centres it on the stance position (in step lengths)
#         self.stride_centre = []
#         for leg in range(4):
#             rel = [self.foot_progress(leg, k / 200) - self.body_progress(k / 200)
#                    for k in range(200)]
#             self.stride_centre.append((max(rel) + min(rel)) / 2)

        # Use the same controller topic as walk_simple.py.
#         self.publisher = self.create_publisher(
#             Float64MultiArray,
#             '/joint_group_controller/commands',
#             10
#         )

        # Standing joint angles: knees bent backwards, as in the old STAND pose
        # (Joint order: BR, BL, FL, FR), refined by IK to the stance below
#         seed = [
#             0.0, -1.4002, -0.46137,  # BR
#             0.0,  1.4002, -0.46137,  # BL
#             0.0,  1.4002, -0.46137,  # FL
#             0.0, -1.4002, -0.46137   # FR
#         ]
#         self.leg_q = [np.array(seed[3 * i:3 * i + 3]) for i in range(4)]
#         self.STAND = self.calculate_stand_pose()

#         self.phase = 0.0
        # Gait amplitude 0..1: ramps up after f and down after s, so the
        # legs start and stop from the standing pose without a jump
#         self.amplitude = 0.0
#         self.walking = False
        # Starts from the measured joint angles (first /joint_states message)
#         self.last_command = None
        # True once the legs have reached STAND; walking waits for it
#         self.standing = False

#         self.create_subscription(
#             JointState, '/joint_states', self.joint_states_callback, 10
#         )

#         self.timer = self.create_timer(
#             self.timer_period, self.update
#         )

#         threading.Thread(
#             target=self.keyboard_input, daemon=True
#         ).start()

#         self.get_logger().info(
#             "Quadruped crawl ready. Enter f to walk, s to stop."
#         )

#     def joint_states_callback(self, msg):
#         if self.last_command is not None:
#             return
#         positions = dict(zip(msg.name, msg.position))
#         if all(name in positions for name in self.joint_names):
#             self.last_command = [positions[name] for name in self.joint_names]

#     def limit_speed(self, targets):
        # Move each joint at most max_joint_speed towards its target per timer tick
#         max_step = self.max_joint_speed * self.timer_period
#         return [
#             last + max(-max_step, min(max_step, target - last))
#             for last, target in zip(self.last_command, targets)
#         ]

#     @staticmethod
#     def smoothstep(u):
#         return 3.0 * u**2 - 2.0 * u**3

#     def is_front(self, leg):
#         return leg in (2, 3)

#     def is_right(self, leg):
#         return leg in (0, 3)

#     def nominal_foot(self, leg):
#         y = -self.stance_width if self.is_right(leg) else self.stance_width
#         return np.array([self.hip_x[leg] + self.stance_x, y, -self.body_height])

#     def body_progress(self, phase):
#         """Body travel (in step lengths) since the start of the cycle."""
#         if phase < 0.25:
#             return 0.5 * self.smoothstep(phase / 0.25)
#         if phase < 0.5:
#             return 0.5
#         if phase < 0.75:
#             return 0.5 + 0.5 * self.smoothstep((phase - 0.5) / 0.25)
#         return 1.0

#     def foot_progress(self, leg, phase):
#         """Toe travel over the ground (in step lengths) since the start of the cycle."""
#         u = (phase - self.swing_start[leg]) / self.swing_time
#         if u <= 0.0:
#             return 0.0
#         if u >= 1.0:
#             return 1.0
        # Cycloid: lifts off and lands with zero speed
#         return u - math.sin(2 * math.pi * u) / (2 * math.pi)

#     def body_sway(self, phase):
#         """Sideways body offset (m, +y = left): right while the left legs step, left
#         while the right legs step, moving across while all four feet are down."""
#         sway = self.sway * self.amplitude
#         if phase < 0.25:
#             return sway - 2 * sway * self.smoothstep(phase / 0.25)
#         if phase < 0.5:
#             return -sway
#         if phase < 0.75:
#             return -sway + 2 * sway * self.smoothstep((phase - 0.5) / 0.25)
#         return sway

#     def foot_target(self, leg, phase):
#         """Toe position relative to the body for the cycle phase."""
#         length = self.step_length * self.amplitude
#         height = self.step_height * self.amplitude

#         dx = length * (self.foot_progress(leg, phase) - self.body_progress(phase)
#                        - self.stride_centre[leg])
#         dz = 0.0
#         u = (phase - self.swing_start[leg]) / self.swing_time
#         if 0.0 < u < 1.0:
#             dz = height * (1 - math.cos(2 * math.pi * u)) / 2
        # Moving the body sideways = moving every foot the other way
#         dy = -self.body_sway(phase)

#         return self.nominal_foot(leg) + np.array([dx, dy, dz])

#     def ik_for_leg(self, leg, target):
        # Start from the leg's last solution so it keeps the same knee bend
#         self.leg_q[leg] = self.legs[leg].ik(target, self.leg_q[leg])
#         return list(self.leg_q[leg])

#     def calculate_stand_pose(self):
#         stand = []
#         for leg in range(4):
#             stand.extend(self.ik_for_leg(leg, self.nominal_foot(leg)))
#         return stand

#     def update(self):
#         if self.last_command is None:
            # Wait for the first /joint_states so the legs start from where they are
#             return

#         if not self.standing:
            # Stand up slowly first
#             positions = self.limit_speed(self.STAND)
#             self.standing = max(abs(a - b) for a, b in zip(positions, self.STAND)) < 1e-6
#             if self.standing:
#                 self.get_logger().info("Standing.")
#         else:
            # Ramp the gait in and out over one cycle
#             ramp = self.timer_period / self.cycle_time
#             if self.walking:
#                 self.amplitude = min(1.0, self.amplitude + ramp)
#             else:
#                 self.amplitude = max(0.0, self.amplitude - ramp)

#             if self.walking or self.amplitude > 0.0:
#                 self.phase = (self.phase + self.timer_period / self.cycle_time) % 1.0

#             positions = []
#             for leg in range(4):
#                 positions.extend(self.ik_for_leg(leg, self.foot_target(leg, self.phase)))

#         msg = Float64MultiArray()
#         msg.data = [float(v) for v in positions]
#         self.publisher.publish(msg)
#         self.last_command = list(positions)

#     def keyboard_input(self):
#         while rclpy.ok():
#             try:
#                 command = input("Command (f=walk, s=stop): ").strip().lower()
#             except EOFError:
#                 break

#             if command == 'f':
#                 self.walking = True
#                 self.get_logger().info(
#                     "Walking." if self.standing else "Walking once standing.")
#             elif command == 's':
#                 self.walking = False
#                 self.get_logger().info("Stopping; legs return to the standing pose.")
#             else:
#                 print("Enter f or s.")


# def main(args=None):
#     rclpy.init(args=args)
#     node = QuadCrawl()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class QuadCrawl(Node):

    def __init__(self):
        super().__init__('quad_crawl')

        # Robot geometry (metres)
        self.L1 = 0.050
        self.L2 = 0.135
        self.L3 = 0.1575

        # Foot-path parameters (metres)
        self.step_length = 0.010
        self.step_height = 0.010

        # Centre of the foot path in the IK coordinate frame.
        # Required because IK needs absolute foot coordinates.
        self.center_y = 0.010
        self.center_z = 0.150

        # Timing — same style as the hexapod controller
        self.dt = 0.05
        self.step_time = 0.5
        self.step_timer = 0.0
        self.step_index = 0
        self.num_steps = 4

        # Speed limit (from the previous walk_gait): each joint moves at most
        # max_joint_speed towards its target, so standing up and starting to
        # walk are smooth instead of jumping
        self.max_joint_speed = 1.0  # rad/s
        # Starts from the measured joint angles (first /joint_states message)
        self.last_command = None

        # Leg order: BR, BL, FL, FR
        # (Gazebo Classic quadruped.urdf names, same order as config/controllers.yaml)
        self.joint_names = [
            'base_link_Revolute_1', 'ccup_3_Revolute_31', 'thigh_3_Revolute_32',
            'base_link_Revolute_3', 'ccup_2_Revolute_28', 'thigh_2_Revolute_29',
            'base_link_Revolute_4', 'ccup_1_Revolute_25', 'thigh_1_Revolute_26',
            'base_link_Revolute_2', 'ccup_4_Revolute_34', 'thigh_4_Revolute_35'
        ]

        # Two diagonal pairs, offset by half a cycle
        self.phase_offsets = [0, 2, 0, 2]

        self.STAND = [
            0.0, -1.4002, -0.46137,  # BR
            0.0,  1.4002, -0.46137,  # BL
            0.0,  1.4002, -0.46137,  # FL
            0.0, -1.4002, -0.46137   # FR
        ]

        # real:=false (default): Gazebo. Commands go to the leg controller, and the
        #   first /joint_states message (from Gazebo) gives the start angles.
        # real:=true: real robot, no Gazebo. The angles are published directly on
        #   /joint_states (read by esp.py and robot_state_publisher), starting from
        #   the zero pose (all servos at 90 degrees).
        self.real = self.declare_parameter('real', False).value

        if self.real:
            self.publisher = self.create_publisher(JointState, '/joint_states', 10)
            self.last_command = [0.0] * len(self.joint_names)
        else:
            self.publisher = self.create_publisher(
                Float64MultiArray,
                '/joint_group_controller/commands',
                10
            )
            self.create_subscription(
                JointState, '/joint_states', self.joint_states_callback, 10
            )

        self.command = 's'

        self.timer = self.create_timer(self.dt, self.update)

        threading.Thread(
            target=self.keyboard_input,
            daemon=True
        ).start()

        self.get_logger().info(
            'Quadruped controller ready (%s): f = move, s = stand'
            % ('real robot, publishing /joint_states' if self.real else 'Gazebo')
        )

    # ---------- Speed limit ----------

    def joint_states_callback(self, msg):
        if self.last_command is not None:
            return
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.last_command = [positions[name] for name in self.joint_names]

    def limit_speed(self, targets):
        # Move each joint at most max_joint_speed towards its target per timer tick
        max_step = self.max_joint_speed * self.dt
        return [
            last + max(-max_step, min(max_step, target - last))
            for last, target in zip(self.last_command, targets)
        ]

    # ---------- Inverse kinematics ----------

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

    # ---------- Leg helpers ----------

    def is_right(self, leg):
        return leg in (0, 3)

    def ik_for_leg(self, leg, x, y, z):
        if self.is_right(leg):
            return self.IK_right(x, y, z)
        return self.IK_left(x, y, z)

    # ---------- Four calculated path points ----------

    def foot_points(self, leg):
        x = 0.05 if self.is_right(leg) else -0.05

        yc = self.center_y
        zc = self.center_z
        L = self.step_length
        H = self.step_height

        # A = centre, B/C = horizontal endpoints, D = raised point.
        # Lower z means lifting the foot in this robot's coordinate setup.
        A = (x, yc, zc)
        B = (x, yc - L, zc)
        D = (x, yc, zc - H)
        C = (x, yc + L, zc)

        # Repeating order: A -> B -> D -> C -> A
        return [A, B, D, C]

    # ---------- Gait update ----------

    def update_gait(self):
        if self.command != 'f':
            return

        self.step_timer += self.dt

        if self.step_timer >= self.step_time:
            self.step_timer = 0.0
            self.step_index = (
                self.step_index + 1
            ) % self.num_steps

    # ---------- Main loop ----------

    def update(self):
        if self.last_command is None:
            # Wait for the first /joint_states so the legs start from where they are
            return

        self.update_gait()

        if self.command == 's':
            positions = list(self.STAND)

        else:
            positions = []

            for leg in range(4):
                idx = (
                    self.step_index + self.phase_offsets[leg]
                ) % self.num_steps

                x, y, z = self.foot_points(leg)[idx]
                q1, q2, q3 = self.ik_for_leg(leg, x, y, z)

                positions.extend([q1, q2, q3])

        positions = self.limit_speed(positions)

        if self.real:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = [float(v) for v in positions]
        else:
            msg = Float64MultiArray()
            msg.data = [float(v) for v in positions]
        self.publisher.publish(msg)
        self.last_command = list(positions)

    # ---------- Keyboard input ----------

    def keyboard_input(self):
        while rclpy.ok():
            try:
                cmd = input('Command (f=move, s=stand): ').strip().lower()
            except EOFError:
                break

            if cmd == 'f':
                self.step_index = 0
                self.step_timer = 0.0
                self.command = 'f'
                self.get_logger().info('Path cycling started.')

            elif cmd == 's':
                self.command = 's'
                self.get_logger().info('Standing pose commanded.')

            else:
                print("Enter 'f' or 's'.")


def main(args=None):
    rclpy.init(args=args)
    node = QuadCrawl()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
