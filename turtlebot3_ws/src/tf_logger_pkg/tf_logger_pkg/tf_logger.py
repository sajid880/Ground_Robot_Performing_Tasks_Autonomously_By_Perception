#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import math
import csv
import os
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import TransformException

CSV_PATH = os.path.expanduser('~/tf_log.csv')

def quaternion_to_yaw(x, y, z, w):
    """
    Convert quaternion to yaw (z-axis rotation).
    Formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    """
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))

class TFLoggerNode(Node):
    def __init__(self):
        super().__init__('tf_logger_node')
        self.get_logger().info('TF Logger node started, writing to: %s' % CSV_PATH)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        # ensure header exists
        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time_sec', 'x', 'y', 'yaw_rad'])

    def timer_callback(self):
        # Try common base frame names in order
        source = 'base_link'
        fallback_source = 'base_footprint'
        target = 'odom'

        now = self.get_clock().now()
        # Use latest transform (time=Time())
        try:
            trans: TransformStamped = self.buffer.lookup_transform(target, source, Time())
        except TransformException:
            try:
                trans: TransformStamped = self.buffer.lookup_transform(target, fallback_source, Time())
                source = fallback_source
            except TransformException as e:
                # nothing available yet
                self.get_logger().debug('Transform lookup failed: %s' % str(e))
                return

        # translation
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        # quaternion
        q = trans.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        # timestamp: use header stamp if available, else now
        try:
            stamp = trans.header.stamp
            # convert to seconds (stamp.sec + stamp.nanosec * 1e-9)
            time_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            time_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9

        # write to CSV (append)
        try:
            with open(CSV_PATH, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([f'{time_sec:.9f}', f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
        except Exception as e:
            self.get_logger().error('Failed to write CSV: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = TFLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down TF logger node.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
