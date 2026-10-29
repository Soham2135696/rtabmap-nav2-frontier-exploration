# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial


# class SerialBridgeLeg1(Node):

#     def __init__(self):
#         super().__init__('serial_bridge_leg1')

#         # -------- SERIAL SETUP --------
#         try:
#             self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#             self.get_logger().info("✅ Serial connected")
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial error: {e}")
#             self.ser = None

#         # -------- SUBSCRIBER --------
#         self.sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.callback,
#             10
#         )

#         # -------- Joint mapping --------
#         self.joint_map = {
#             "CCUP1": 0,
#             "THIGH1": 1,
#             "FOOT1": 2
#         }

#     # -------- CALLBACK --------
#     def callback(self, msg: JointState):

#         try:
#             # Get indices dynamically (safe)
#             idx_ccup  = msg.name.index("CCUP1")
#             idx_thigh = msg.name.index("THIGH1")
#             idx_foot  = msg.name.index("FOOT1")

#             # Extract angles (rad)
#             ccup  = msg.position[idx_ccup]
#             thigh = msg.position[idx_thigh]
#             foot  = msg.position[idx_foot]

#             # Convert to degrees
#             ccup_d  = math.degrees(ccup)
#             thigh_d = 90-math.degrees(thigh)
#             foot_d  = math.degrees(foot)

#             # Clamp (servo safety)
#             def clamp(x):
#                 return max(0, min(180, x))

#             ccup_d  = clamp(ccup_d)
#             thigh_d = clamp(thigh_d)
#             foot_d  = clamp(foot_d)

#             # Format message
#             data = f"{ccup_d:.2f} {thigh_d:.2f} {foot_d:.2f}\n"

#             # Send
#             if self.ser:
#                 self.ser.write(data.encode())

#             # Debug (optional)
#             self.get_logger().info(f"Sent: {data.strip()}")

#         except ValueError:
#             # Joint not found in message
#             pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialBridgeLeg1()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial


# class SerialBridgeLeg12(Node):

#     def __init__(self):
#         super().__init__('serial_bridge_leg12')

#         # -------- SERIAL SETUP --------
#         try:
#             self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#             self.get_logger().info("✅ Serial connected")
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial error: {e}")
#             self.ser = None

#         # -------- SUBSCRIBER --------
#         self.sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.callback,
#             10
#         )

#     # -------- CALLBACK --------
#     def callback(self, msg: JointState):

#         try:
#             # -------- LEG 1 --------
#             i1 = msg.name.index("CCUP1")
#             i2 = msg.name.index("THIGH1")
#             i3 = msg.name.index("FOOT1")

#             ccup1  = math.degrees(msg.position[i1])
#             thigh1 = 90 - math.degrees(msg.position[i2])
#             foot1  = math.degrees(msg.position[i3])

#             # -------- LEG 2 --------
#             i4 = msg.name.index("CCUP2")
#             i5 = msg.name.index("THIGH2")
#             i6 = msg.name.index("FOOT2")

#             ccup2  = math.degrees(msg.position[i4])
#             thigh2 = 90 - math.degrees(msg.position[i5])
#             foot2  = math.degrees(msg.position[i6])

#             # -------- CLAMP --------
#             def clamp(x):
#                 return max(0, min(180, x))

#             ccup1, thigh1, foot1 = clamp(ccup1), clamp(thigh1), clamp(foot1)
#             ccup2, thigh2, foot2 = clamp(ccup2), clamp(thigh2), clamp(foot2)

#             # -------- FORMAT --------
#             # Format: L1 → L2
#             data = (
#                 f"{ccup2:.2f} {thigh2:.2f} {foot2:.2f} "
#                 f"{ccup1:.2f} {thigh1:.2f} {foot1:.2f}\n"
#             )

#             # -------- SEND --------
#             if self.ser:
#                 self.ser.write(data.encode())

#             # Debug
#             self.get_logger().info(f"Sent: {data.strip()}")

#         except ValueError:
#             pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialBridgeLeg12()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial


# class SerialBridgeLeg12(Node):

#     def __init__(self):
#         super().__init__('serial_bridge_leg12')

#         # -------- SERIAL SETUP --------
#         try:
#             self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#             self.get_logger().info("✅ Serial connected")
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial error: {e}")
#             self.ser = None

#         # -------- SUBSCRIBER --------
#         self.sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.callback,
#             10
#         )

#     # -------- UTILS --------
#     def clamp(self, x):
#         # return max(0, min(180, x))
#         return x

#     # -------- CALLBACK --------
#     def callback(self, msg: JointState):

#         try:
#             # -------- LEG 1 --------
#             ccup1  = math.degrees(msg.position[msg.name.index("CCUP1")])
#             thigh1 = math.degrees(msg.position[msg.name.index("THIGH1")])
#             foot1  = math.degrees(msg.position[msg.name.index("FOOT1")])

#             # -------- LEG 2 --------
#             ccup2  = math.degrees(msg.position[msg.name.index("CCUP2")])
#             thigh2 = math.degrees(msg.position[msg.name.index("THIGH2")])
#             foot2  = math.degrees(msg.position[msg.name.index("FOOT2")])

#             # -------- LEG 3 --------
#             ccup3  = math.degrees(msg.position[msg.name.index("CCUP3")])
#             thigh3 = math.degrees(msg.position[msg.name.index("THIGH3")])
#             foot3  = math.degrees(msg.position[msg.name.index("FOOT3")])

#             # -------- LEG 4 --------
#             ccup4  = math.degrees(msg.position[msg.name.index("CCUP4")])
#             thigh4 = math.degrees(msg.position[msg.name.index("THIGH4")])
#             foot4  = math.degrees(msg.position[msg.name.index("FOOT4")])

#             # -------- CLAMP --------
#             ccup1, thigh1, foot1 = self.clamp(ccup1), self.clamp(thigh1), self.clamp(foot1)
#             ccup2, thigh2, foot2 = self.clamp(ccup2), self.clamp(thigh2), self.clamp(foot2)
#             ccup3, thigh3, foot3 = self.clamp(ccup3), self.clamp(thigh3), self.clamp(foot3)
#             ccup4, thigh4, foot4 = self.clamp(ccup4), self.clamp(thigh4), self.clamp(foot4)           

#             # -------- FORMAT --------
#             # Order: L1 → L4
#             data = (
#                 f"{ccup1:.2f} {thigh1:.2f} {foot1:.2f} "
#                 f"{ccup2:.2f} {thigh2:.2f} {foot2:.2f} "
#                 f"{ccup3:.2f} {thigh3:.2f} {foot3:.2f} "
#                 f"{ccup4:.2f} {thigh4:.2f} {foot4:.2f}\n"
#             )

#             # -------- SEND --------
#             if self.ser:
#                 self.ser.write(data.encode())

#             self.get_logger().info(f"Sent: {data.strip()}")

#         except ValueError:
#             # Happens when joint names not yet published
#             pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialBridgeLeg12()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial


# class SerialBridgeLeg1(Node):

#     def __init__(self):
#         super().__init__('serial_bridge_leg1')

#         # -------- SERIAL SETUP --------
#         try:
#             self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#             self.get_logger().info("✅ Serial connected")
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial error: {e}")
#             self.ser = None

#         # -------- SUBSCRIBER --------
#         self.sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.callback,
#             10
#         )

#         # -------- Joint mapping --------
#         self.joint_map = {
#             "CCUP1": 0,
#             "THIGH1": 1,
#             "FOOT1": 2
#         }

#     # -------- CALLBACK --------
#     def callback(self, msg: JointState):

#         try:
#             # Get indices dynamically (safe)
#             idx_ccup  = msg.name.index("CCUP1")
#             idx_thigh = msg.name.index("THIGH1")
#             idx_foot  = msg.name.index("FOOT1")

#             # Extract angles (rad)
#             ccup  = msg.position[idx_ccup]
#             thigh = msg.position[idx_thigh]
#             foot  = msg.position[idx_foot]

#             # Convert to degrees
#             ccup_d  = math.degrees(ccup)
#             thigh_d = 90-math.degrees(thigh)
#             foot_d  = math.degrees(foot)

#             # Clamp (servo safety)
#             def clamp(x):
#                 return max(0, min(180, x))

#             ccup_d  = clamp(ccup_d)
#             thigh_d = clamp(thigh_d)
#             foot_d  = clamp(foot_d)

#             # Format message
#             data = f"{ccup_d:.2f} {thigh_d:.2f} {foot_d:.2f}\n"

#             # Send
#             if self.ser:
#                 self.ser.write(data.encode())

#             # Debug (optional)
#             self.get_logger().info(f"Sent: {data.strip()}")

#         except ValueError:
#             # Joint not found in message
#             pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialBridgeLeg1()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial


# class SerialBridgeLeg12(Node):

#     def __init__(self):
#         super().__init__('serial_bridge_leg12')

#         # -------- SERIAL SETUP --------
#         try:
#             self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#             self.get_logger().info("✅ Serial connected")
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial error: {e}")
#             self.ser = None

#         # -------- SUBSCRIBER --------
#         self.sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.callback,
#             10
#         )

#     # -------- CALLBACK --------
#     def callback(self, msg: JointState):

#         try:
#             # -------- LEG 1 --------
#             i1 = msg.name.index("CCUP1")
#             i2 = msg.name.index("THIGH1")
#             i3 = msg.name.index("FOOT1")

#             ccup1  = math.degrees(msg.position[i1])
#             thigh1 = 90 - math.degrees(msg.position[i2])
#             foot1  = math.degrees(msg.position[i3])

#             # -------- LEG 2 --------
#             i4 = msg.name.index("CCUP2")
#             i5 = msg.name.index("THIGH2")
#             i6 = msg.name.index("FOOT2")

#             ccup2  = math.degrees(msg.position[i4])
#             thigh2 = 90 - math.degrees(msg.position[i5])
#             foot2  = math.degrees(msg.position[i6])

#             # -------- CLAMP --------
#             def clamp(x):
#                 return max(0, min(180, x))

#             ccup1, thigh1, foot1 = clamp(ccup1), clamp(thigh1), clamp(foot1)
#             ccup2, thigh2, foot2 = clamp(ccup2), clamp(thigh2), clamp(foot2)

#             # -------- FORMAT --------
#             # Format: L1 → L2
#             data = (
#                 f"{ccup2:.2f} {thigh2:.2f} {foot2:.2f} "
#                 f"{ccup1:.2f} {thigh1:.2f} {foot1:.2f}\n"
#             )

#             # -------- SEND --------
#             if self.ser:
#                 self.ser.write(data.encode())

#             # Debug
#             self.get_logger().info(f"Sent: {data.strip()}")

#         except ValueError:
#             pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialBridgeLeg12()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial


class SerialBridgeLeg12(Node):

    def __init__(self):
        super().__init__('serial_bridge_leg12')

        # -------- SERIAL SETUP --------
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("✅ Serial connected")
        except Exception as e:
            self.get_logger().error(f"❌ Serial error: {e}")
            self.ser = None

        # -------- SUBSCRIBER --------
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

    # -------- UTILS --------
    def clamp(self, x):
        # return max(0, min(180, x))
        return x

    # -------- CALLBACK --------
    def callback(self, msg: JointState):

        try:
            # -------- LEG 1 --------
            ccup1  = math.degrees(msg.position[msg.name.index("CCUP1")]) +90
            thigh1 = -math.degrees(msg.position[msg.name.index("THIGH1")]) +90
            foot1  = -math.degrees(msg.position[msg.name.index("FOOT1")]) +90

            # -------- LEG 2 --------
            ccup2  = math.degrees(msg.position[msg.name.index("CCUP2")]) +90
            thigh2 = -math.degrees(msg.position[msg.name.index("THIGH2")]) +90
            foot2  = math.degrees(msg.position[msg.name.index("FOOT2")]) +90

            # -------- LEG 3 --------
            ccup3  = -math.degrees(msg.position[msg.name.index("CCUP3")]) +90
            thigh3 = -math.degrees(msg.position[msg.name.index("THIGH3")]) +90
            foot3  = math.degrees(msg.position[msg.name.index("FOOT3")]) +90

            # -------- LEG 4 --------
            ccup4  = -math.degrees(msg.position[msg.name.index("CCUP4")]) +90
            thigh4 = -math.degrees(msg.position[msg.name.index("THIGH4")]) +90 
            foot4  = -math.degrees(msg.position[msg.name.index("FOOT4")]) +90

            # -------- CLAMP --------
            ccup1, thigh1, foot1 = self.clamp(ccup1), self.clamp(thigh1), self.clamp(foot1)
            ccup2, thigh2, foot2 = self.clamp(ccup2), self.clamp(thigh2), self.clamp(foot2)
            ccup3, thigh3, foot3 = self.clamp(ccup3), self.clamp(thigh3), self.clamp(foot3)
            ccup4, thigh4, foot4 = self.clamp(ccup4), self.clamp(thigh4), self.clamp(foot4)           

            # -------- FORMAT --------
            # Order: L1 → L4
            data = (
                f"{ccup1:.2f} {thigh1:.2f} {foot1:.2f} "
                f"{ccup2:.2f} {thigh2:.2f} {foot2:.2f} "
                f"{ccup3:.2f} {thigh3:.2f} {foot3:.2f} "
                f"{ccup4:.2f} {thigh4:.2f} {foot4:.2f}\n"
            )

            # -------- SEND --------
            if self.ser:
                self.ser.write(data.encode())

            self.get_logger().info(f"Sent: {data.strip()}")

        except ValueError:
            # Happens when joint names not yet published
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeLeg12()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()