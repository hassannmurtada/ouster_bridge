#!/usr/bin/env python3
"""
Tail live_pose.csv produced by `ouster-cli localize`
and broadcast each pose as TF (map→base_link) and Odometry.
"""

import time, csv, os, rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import rclpy.utilities

def tail_file(path):
    """Generator that yields one CSV row (list[str]) as soon as it appears."""
    with open(path, 'r') as f:
        f.seek(0, os.SEEK_END)          # jump to file end
        reader = csv.reader(f)
        while True:
            line = f.readline()
            if line:
                yield next(csv.reader([line]))
            else:
                time.sleep(0.02)

def odom_from_tf(tf_msg):
    odom = Odometry()
    odom.header = tf_msg.header
    odom.child_frame_id = tf_msg.child_frame_id
    odom.pose.pose.position.x = tf_msg.transform.translation.x
    odom.pose.pose.position.y = tf_msg.transform.translation.y
    odom.pose.pose.position.z = tf_msg.transform.translation.z
    odom.pose.pose.orientation = tf_msg.transform.rotation
    return odom

def main():
    import argparse, sys
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default='live_pose.csv',
                    help='CSV produced by ouster-cli localize save_trajectory')
    ap.add_argument('--publish_odom', action='store_true',
                    help='also publish /odom nav_msgs/Odometry')
    clean_args = rclpy.utilities.remove_ros_args(sys.argv)
    args = ap.parse_args(clean_args[1:])   # skip program name

    rclpy.init()
    node = rclpy.create_node('ouster_csv_tf_bridge')
    br   = TransformBroadcaster(node)
    if args.publish_odom:
        odom_pub = node.create_publisher(Odometry, 'odom', 10)

    node.get_logger().info(f"Tail-bridging {args.file}")

    for row in tail_file(args.file):
        try:
            t,x,y,z,qx,qy,qz,qw = map(float, row)
        except ValueError:
            node.get_logger().warn(f"Bad row skipped: {row}")
            continue

        tf_msg = TransformStamped()
        tf_msg.header.stamp = node.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id  = 'base_link'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        br.sendTransform(tf_msg)
        if args.publish_odom:
            odom_pub.publish(odom_from_tf(tf_msg))

        rclpy.spin_once(node, timeout_sec=0.0)  # let callbacks run briefly

if __name__ == '__main__':
    main()

