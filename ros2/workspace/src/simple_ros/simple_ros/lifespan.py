import argparse
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from std_msgs.msg import String
from simple_ros.deadline import Talker, Listener


# qos event 처리 callback을 받아 처리하는 publisher class 

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('lifespan', type=int, help='lifespan QoS setting milliseconds')
    parser.add_argument('--history', type=int, default=10, help='history depth QoS setting milliseconds')
    parser.add_argument('--publish-count', type=int, default=10, help='publish count before stopping')
    parser.add_argument('--subscribe-after', type=int, default=2500, help='sub create after startup')
    return parser.parse_args()

def main(args=None):
    parsed_args = parse_args()
    rclpy.init(args=args)
    topic = 'qos_lifespan_chatter'
    lifespan = Duration(seconds=parsed_args.lifespan/1000.0)
    qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, 
                             durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, lifespan=lifespan)

    listener = Listener(topic, qos_profile, event_callbacks=None, defer_subscribe=True)
    talker = Talker(topic, qos_profile, event_callbacks=None, publish_count=parsed_args.publish_count)
    subscribe_timer = listener.create_timer(parsed_args.subscribe_after/1000.0, lambda: listener.start_listening())

    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(talker)
    try:
        executor.spin()
    except KeyboardInterrupt:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
