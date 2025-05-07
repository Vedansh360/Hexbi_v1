#!/usr/bin/env python

import rospy
import redis
import base64
import numpy as np
import cv2
import json
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

def main():
    rospy.init_node('redis_image_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(10)  # Match camera FPS

    redis_client = redis.Redis(host='localhost', port=6379, db=0)
    bridge = CvBridge()

    seq = 0
    frame_id = "camera"

    rospy.loginfo("Starting Redis Image Publisher Node")

    while not rospy.is_shutdown():
        try:
            data = redis_client.get('camera_frame')
            if data:
                json_data = json.loads(data)

                frame_b64 = json_data.get("frame", "")
                timestamp = float(json_data.get("timestamp", rospy.Time.now().to_sec()))

                frame_bytes = base64.b64decode(frame_b64)
                np_arr = np.frombuffer(frame_bytes, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

                    # Fill full header info
                    header = Header()
                    header.seq = seq
                    header.stamp = rospy.Time.now()
                    header.frame_id = frame_id

                    msg.header = header

                    pub.publish(msg)
                    seq += 1
        except Exception as e:
            rospy.logwarn(f"Error reading/publishing frame: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
