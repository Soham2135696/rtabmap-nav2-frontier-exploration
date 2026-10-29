#!/usr/bin/env python3
"""Odometry for the legs-only simulation (urdf/quadruped.urdf).

The wheeled rover gets /odom and TF odom -> base_footprint from Gazebo's diff_drive plugin.
The quadruped has no wheels, so Gazebo's p3d plugin publishes the ground-truth pose of
base_footprint on /ground_truth (frame world), and this node turns it into the same
/odom and TF the rest of the stack expects.

The pose is flattened to x, y and yaw (z, roll and pitch dropped), like diff_drive's,
so the body bouncing while walking does not tilt the map. odom coincides with world,
as with diff_drive's ground-truth odometry.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthOdom(Node):

    def __init__(self):
        super().__init__('ground_truth_odom')
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'ground_truth', self.callback, 10)

    def callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = p.x
        odom.pose.pose.position.y = p.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # p3d gives the twist in the world frame; Odometry wants it in child_frame_id
        v = msg.twist.twist.linear
        c, s = math.cos(yaw), math.sin(yaw)
        odom.twist.twist.linear.x = c * v.x + s * v.y
        odom.twist.twist.linear.y = -s * v.x + c * v.y
        odom.twist.twist.angular.z = msg.twist.twist.angular.z
        self.pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = self.base_frame
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
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
