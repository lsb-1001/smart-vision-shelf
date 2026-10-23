"""Simulation-only Nav2 goal followed by marker identity verification."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid

from shelf_core import Registry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('location',choices=['A','B','C','DELIVERY'])
    parser.add_argument('--initialize',action='store_true',
                        help='ONLY directly after a fresh sim spawn at x=2.2,y=0,yaw=0')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        parser.error('Simulation isolation requires ROS_DOMAIN_ID=87')
    registry = Registry.load(Path(__file__).with_name('sim')/'locations.sim.json')
    target = registry.lookup(args.location,'gazebo_shelf_demo_v1')
    if target['status'] != 'ready' or not registry.data.get('simulation_only'):
        parser.error('Generated simulation coordinates required')
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String
    from lifecycle_msgs.srv import GetState

    rclpy.init()
    node = Node('shelf_sim_mission',parameter_overrides=[Parameter('use_sim_time',value=True)])
    nav = ActionClient(node,NavigateToPose,'navigate_to_pose')
    pose_pub = node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    request_pub = node.create_publisher(String,'/shelf_vision/request',10)
    request_id = 'nav-'+uuid.uuid4().hex
    verification = {}
    def receive(message):
        data = json.loads(message.data)
        if data.get('request_id') == request_id:
            if data.get('ok') is False:
                verification.update(state='request_error',error=data.get('error'))
            elif 'state' in data and not data.get('active',True):
                verification.update(data)
    result_sub = node.create_subscription(String,'/shelf_vision/result',receive,10)
    reply_sub = node.create_subscription(String,'/shelf_vision/reply',receive,10)
    handle, completed = None, False
    def spin_until(predicate, seconds, error):
        deadline = time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.1)
        if not predicate():
            raise RuntimeError(error)
    try:
        spin_until(lambda:node.get_clock().now().nanoseconds>0,10,'No simulation /clock')
        spin_until(lambda:request_pub.get_subscription_count()>0,5,'Start shelf_ros.py first')
        # Discovery of an action server does not mean its lifecycle is active.
        # Wait before initialization/sending goals to avoid startup rejection.
        for name in ['map_server','amcl','planner_server','controller_server','bt_navigator']:
            client = node.create_client(GetState,'/'+name+'/get_state')
            try:
                deadline = time.monotonic()+45
                active = False
                while time.monotonic()<deadline and not active:
                    if not client.wait_for_service(timeout_sec=0.5):
                        continue
                    pending = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(node,pending,timeout_sec=1)
                    active = bool(pending.done() and pending.result()
                                  and pending.result().current_state.id == 3)
                    if not pending.done():
                        pending.cancel()
                    if not active:
                        rclpy.spin_once(node,timeout_sec=0.2)
                if not active:
                    raise RuntimeError(name+' lifecycle did not become active')
            finally:
                node.destroy_client(client)
        print('Nav2 lifecycle nodes active.',flush=True)
        if args.initialize:
            spin_until(lambda:pose_pub.get_subscription_count()>0,15,'AMCL initialpose subscriber missing')
            for _ in range(4):
                message = PoseWithCovarianceStamped()
                message.header.frame_id = 'map'
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.pose.position.x = 2.2
                message.pose.pose.orientation.w = 1.0
                message.pose.covariance[0] = message.pose.covariance[7] = 0.0025
                message.pose.covariance[35] = 0.01
                pose_pub.publish(message)
                rclpy.spin_once(node,timeout_sec=0.2)
            print('Initialized AMCL to fresh-spawn pose (2.2,0,0).',flush=True)
        if not nav.wait_for_server(timeout_sec=20):
            raise RuntimeError('Nav2 action server unavailable; start shelf_nav.launch.py')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        pose = target['approach_pose']
        goal.pose.pose.position.x,goal.pose.pose.position.y = pose['x'],pose['y']
        goal.pose.pose.orientation.z = math.sin(pose['yaw']/2)
        goal.pose.pose.orientation.w = math.cos(pose['yaw']/2)
        print(f'NAVIGATING to {args.location}: {pose}',flush=True)
        future = nav.send_goal_async(goal)
        spin_until(future.done,10,'Goal acknowledgement timeout')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected goal')
        result_future = handle.get_result_async()
        spin_until(result_future.done,180,'Navigation timeout; cancellation requested')
        completed = True
        if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Nav2 did not succeed; marker verification not started')
        print('NAV2 SUCCEEDED. Requesting fresh marker verification.',flush=True)
        request_pub.publish(String(data=json.dumps(dict(op='verify',location=args.location,
                                                         request_id=request_id,navigation_arrived=True))))
        spin_until(lambda:bool(verification),13,'No marker verification result')
        print(json.dumps(verification,indent=2),flush=True)
        if verification.get('state') != 'verified':
            raise RuntimeError('Marker verification failed; no cargo/mission completion asserted')
        print('NAVIGATION + MARKER ID VERIFIED. Await worker loading/unloading separately.',flush=True)
    finally:
        if handle is not None and not completed:
            cancelled = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,cancelled,timeout_sec=2)
        nav.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
