#!/usr/bin/env python3
"""使用可复现的模拟数据验证乱序、部分更新、非法消息和暂停恢复。"""
import argparse
import math
import random
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', type=int, default=7)
    parser.add_argument('--hz', type=float, default=50)
    parser.add_argument('--topic', default='/joint_states')
    parser.add_argument('--shuffle', action='store_true')
    parser.add_argument('--partial', action='store_true', help='运行三秒后仅更新奇数编号的关节')
    parser.add_argument('--mixed', action='store_true', help='发布左右旋转关节和直线关节的混合示例')
    parser.add_argument('--invalid', choices=['empty', 'mismatch', 'nan', 'inf', 'duplicate'])
    parser.add_argument('--pause-after', type=float, default=0)
    parser.add_argument('--pause-for', type=float, default=3)
    args, ros_args = parser.parse_known_args()
    if args.count < 1 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error('count and hz must be positive')
    rclpy.init(args=ros_args)
    node = Node('mock_joint_states_publisher')
    pub = node.create_publisher(JointState, args.topic, qos_profile_sensor_data)
    start = time.monotonic()
    rng = random.Random(42)

    def publish():
        elapsed = time.monotonic() - start
        if args.pause_after > 0 and args.pause_after <= elapsed < args.pause_after + args.pause_for:
            return
        names = ['left_joint1', 'right_joint1', 'slide'] if args.mixed else [f'joint{i+1}' for i in range(args.count)]
        pairs = [(name, 0.25 + 0.2 * math.sin(elapsed) if name == 'slide' else math.sin(elapsed + i)) for i, name in enumerate(names)]
        if args.partial and elapsed > 3:
            pairs = pairs[::2]
        if args.shuffle:
            rng.shuffle(pairs)
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = [x[0] for x in pairs]
        msg.position = [x[1] for x in pairs]
        if args.invalid == 'empty':
            msg.position = []
        elif args.invalid == 'mismatch':
            msg.position.append(0.0)
        elif args.invalid in ('nan', 'inf'):
            msg.position[0] = float(args.invalid)
        elif args.invalid == 'duplicate':
            msg.name.append(msg.name[0])
            msg.position.append(99.0)
        pub.publish(msg)

    node.create_timer(1 / args.hz, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
