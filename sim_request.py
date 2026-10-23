"""Send a request and wait for its response on the isolated simulation domain."""
import argparse
import json
import os
import time
import uuid
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('operation',choices=['lookup','verify'])
    parser.add_argument('location',choices=['A','B','C','DELIVERY'])
    parser.add_argument('--assume-arrived',action='store_true',
                        help='Explicitly simulate the navigation-arrived event; no autonomous run')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        parser.error('Set ROS_DOMAIN_ID=87')
    if args.operation == 'verify' and not args.assume_arrived:
        parser.error('Use --assume-arrived for parked-camera verification, or sim_mission.py after Nav2 arrival')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    node = Node('shelf_sim_request')
    request_id = 'sim-'+uuid.uuid4().hex
    received = {}
    def receive(message):
        data = json.loads(message.data)
        if data.get('request_id') == request_id:
            if 'ok' in data or not data.get('active',True):
                received.update(data)
    pub = node.create_publisher(String,'/shelf_vision/request',10)
    reply = node.create_subscription(String,'/shelf_vision/reply',receive,10)
    result = node.create_subscription(String,'/shelf_vision/result',receive,10)
    try:
        deadline = time.monotonic()+5
        while pub.get_subscription_count() == 0 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('shelf_ros.py not discovered on domain 87')
        pub.publish(String(data=json.dumps(dict(op=args.operation,location=args.location,
                                                request_id=request_id,navigation_arrived=True))))
        deadline = time.monotonic()+13
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
            if received.get('ok') is False:
                raise RuntimeError(received['error'])
            if args.operation == 'lookup' and received:
                print(json.dumps(received,indent=2)); return
            if 'state' in received:
                print(json.dumps(received,indent=2)); return
        raise RuntimeError('No terminal verification result; inspect shelf_ros output')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
