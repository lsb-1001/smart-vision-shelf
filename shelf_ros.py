"""ROS2 adapter. Topic names are provisional module interfaces, not team agreements."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

from shelf_core import Registry, ArrivalVerifier, frame_freshness
from shelf_view import dictionary, detect


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registry',default=str(Path(__file__).with_name('locations.json')))
    parser.add_argument('--image-topic',default='/camera/image_raw')
    parser.add_argument('--compressed',action='store_true')
    parser.add_argument('--show',action='store_true')
    args, ros_args = parser.parse_known_args()
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    registry = Registry.load(args.registry)
    book = dictionary(cv2,registry.data['dictionary'])

    class ShelfVision(Node):
        def __init__(self):
            super().__init__('shelf_vision')
            self.verifier = ArrivalVerifier(registry)
            self.bridge = CvBridge()
            self.verification_stamp = None
            self.last_source_stamp = None
            self.recent_requests = deque(maxlen=100)
            self.result_pub = self.create_publisher(String,'/shelf_vision/result',10)
            self.observation_pub = self.create_publisher(String,'/shelf_vision/observations',10)
            self.reply_pub = self.create_publisher(String,'/shelf_vision/reply',10)
            self.debug_pub = self.create_publisher(Image,'/shelf_vision/debug',1)
            self.request_sub = self.create_subscription(String,'/shelf_vision/request',self.request,10)
            self.camera_sub = self.create_subscription(
                CompressedImage if args.compressed else Image,args.image_topic,
                self.image,qos_profile_sensor_data)
            self.registry_service = self.create_service(Trigger,'/shelf_vision/registry',self.list_registry)
            self.timer = self.create_timer(0.2,self.health)
            self.get_logger().info('Shelf vision ready. No motor publishers. Waiting for request.')
            if not registry.data['dictionary_confirmed_by_team']:
                self.get_logger().warn('Dictionary is provisional: '+registry.data['dictionary'])

        def publish(self, publisher, result):
            publisher.publish(String(data=json.dumps(result,ensure_ascii=False)))

        def list_registry(self, request, response):
            response.success = True
            response.message = json.dumps([registry.lookup(k) for k in registry.by_key],ensure_ascii=False)
            return response

        def request(self, message):
            request_id = None
            try:
                body = json.loads(message.data)
                request_id = body.get('request_id')
                if not isinstance(request_id,str) or not request_id.strip():
                    raise ValueError('request_id must be a non-empty string')
                if request_id in self.recent_requests:
                    raise ValueError('Duplicate request_id')
                operation = body['op']
                if operation == 'lookup':
                    result = registry.lookup(body['location'],body.get('map_id'))
                elif operation == 'verify':
                    result = self.verifier.begin(body['location'],request_id,time.monotonic(),
                                                 navigation_arrived=body.get('navigation_arrived'))
                    self.verification_stamp = self.get_clock().now().nanoseconds
                    self.last_source_stamp = None
                    self.publish(self.result_pub,result)
                elif operation == 'cancel':
                    result = self.verifier.cancel()
                    self.publish(self.result_pub,result)
                else:
                    raise ValueError('Unknown operation')
                self.recent_requests.append(request_id)
                self.publish(self.reply_pub,dict(request_id=request_id,ok=True,result=result))
            except (ValueError,KeyError,TypeError,AttributeError) as exc:
                self.publish(self.reply_pub,dict(request_id=request_id,ok=False,error=str(exc)))

        def health(self):
            if self.verifier.request_id is not None:
                self.publish(self.result_pub,self.verifier.tick(time.monotonic()))

        def image(self, message):
            try:
                if args.compressed:
                    frame = cv2.imdecode(np.frombuffer(message.data,np.uint8),cv2.IMREAD_COLOR)
                    if frame is None:
                        raise ValueError('Image decode failed')
                else:
                    frame = self.bridge.imgmsg_to_cv2(message,desired_encoding='bgr8')
                corners,ids,_ = detect(cv2,cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),book)
                found = [] if ids is None else [int(i) for i in ids.flatten()]
                stamp = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
                now_ros = self.get_clock().now().nanoseconds
                fresh,age,freshness_reason = frame_freshness(
                    stamp,now_ros,self.verification_stamp,self.last_source_stamp,
                    registry.data['verification']['camera_timeout_seconds'],
                    registry.data['verification'].get('clock_skew_tolerance_seconds',0.1))
                self.last_source_stamp = stamp
                state = self.verifier.observe(found,time.monotonic(),fresh=fresh)
                self.publish(self.observation_pub,dict(
                    ids=found,locations=[registry.by_id[i]['key'] for i in found if i in registry.by_id],
                    unknown_ids=[i for i in found if i not in registry.by_id],
                    source_frame=message.header.frame_id,source_stamp_ns=stamp,
                    source_fresh=fresh,source_age_seconds=age,
                    source_freshness_reason=freshness_reason))
                if self.verifier.request_id is not None:
                    self.publish(self.result_pub,state)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame,corners,ids)
                cv2.putText(frame,f"Target: {state['target']}  {state['state']}",(8,25),
                            cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,0),1)
                out = self.bridge.cv2_to_imgmsg(frame,encoding='bgr8')
                out.header = message.header
                self.debug_pub.publish(out)
                if args.show:
                    cv2.imshow('Shelf camera verification',frame)
                    cv2.waitKey(1)
            except Exception as exc:
                self.publish(self.result_pub,self.verifier.camera_error())
                self.get_logger().error(str(exc))

    rclpy.init(args=ros_args)
    node = ShelfVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
