#joint caliberation testing
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import serial
import math


class JointStateToESP(Node):

    def __init__(self):
        super().__init__("jointstate_to_esp")

        # -------- SERIAL --------
        self.ser = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=115200,
            timeout=1
        )

        # -------- SUBSCRIBE --------
        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )

        self.get_logger().info("JointState → ESP Bridge Started")

        # -------- EXACT JOINT ORDER --------
        self.joint_order = [
            "CCUP1", "THIGH1", "FOOT1",
            "CCUP2", "THIGH2", "FOOT2",
            "CCUP3", "THIGH3", "FOOT3",
            "CCUP4", "THIGH4", "FOOT4",
        ]


    def callback(self, msg):

        joint_map = dict(zip(msg.name, msg.position))

        angles_deg = []

        for i, joint in enumerate(self.joint_order):

            if joint not in joint_map:
                self.get_logger().warn(f"{joint} missing!")
                return

            angle_rad = joint_map[joint]
            angle_deg = math.degrees(angle_rad)

            # shift to servo range
            angle_deg = angle_deg #+ 90

            # ---- APPLY INVERSION HERE ----
            # if joint in ["THIGH2", "THIGH3"]:
            #     angle_deg = angle_deg #+ 270
            # ---- APPLY INVERSION HERE ----
            if joint in ["CCUP1", "CCUP2", "FOOT2", "FOOT3", "THIGH1", "THIGH2", "THIGH3", "THIGH4",]:
                angle_deg = -angle_deg #+ 90
            # clamp
            angle_deg = max(-180.0, min(180.0, angle_deg))

            angles_deg.append(int(round(angle_deg)))

        # -------- SEND --------
        data = " ".join(map(str, angles_deg)) + "\n"

        try:
            self.ser.write(data.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial error: {e}")

        self.get_logger().info(f"Sent: {data.strip()}")


def main(args=None):
    rclpy.init(args=args)

    node = JointStateToESP()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()