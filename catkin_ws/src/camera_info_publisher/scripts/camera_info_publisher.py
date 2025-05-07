#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    rospy.init_node('camera_info_publisher', anonymous=True)
    rate = rospy.Rate(30)  # match your camera FPS

    cam_info = CameraInfo()
    cam_info.header.frame_id = "camera"
    cam_info.height = 480
    cam_info.width = 640

    # Fictitious intrinsic parameters (replace with real calibration if available)
    cam_info.K = [525.0, 0.0, 319.5,
                  0.0, 525.0, 239.5,
                  0.0, 0.0, 1.0]

    cam_info.P = [525.0, 0.0, 319.5, 0.0,
                  0.0, 525.0, 239.5, 0.0,
                  0.0, 0.0, 1.0, 0.0]

    cam_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming no distortion
    cam_info.distortion_model = "plumb_bob"

    while not rospy.is_shutdown():
        cam_info.header.stamp = rospy.Time.now()
        pub.publish(cam_info)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
