#!/usr/bin/env python

import rospy
import redis
import json
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion

def main():
    rospy.init_node('redis_imu_publisher', anonymous=True)
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rate = rospy.Rate(100)  # Adjust based on your IMU data frequency

    redis_client = redis.Redis(host='localhost', port=6379, db=0)

    rospy.loginfo("Starting Redis IMU Publisher Node")

    while not rospy.is_shutdown():
        try:
            imu_json = redis_client.get('imu_data')
            if imu_json:
                imu_data = json.loads(imu_json)

                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = imu_data["header"]["frame_id"]

                imu_msg.orientation = Quaternion(
                    x=imu_data["orientation"]["x"],
                    y=imu_data["orientation"]["y"],
                    z=imu_data["orientation"]["z"],
                    w=imu_data["orientation"]["w"]
                )
                imu_msg.orientation_covariance = imu_data["orientation_covariance"]

                imu_msg.angular_velocity = Vector3(
                    x=imu_data["angular_velocity"]["x"],
                    y=imu_data["angular_velocity"]["y"],
                    z=imu_data["angular_velocity"]["z"]
                )
                imu_msg.angular_velocity_covariance = imu_data["angular_velocity_covariance"]

                imu_msg.linear_acceleration = Vector3(
                    x=imu_data["linear_acceleration"]["x"],
                    y=imu_data["linear_acceleration"]["y"],
                    z=imu_data["linear_acceleration"]["z"]
                )
                imu_msg.linear_acceleration_covariance = imu_data["linear_acceleration_covariance"]

                pub.publish(imu_msg)

        except Exception as e:
            rospy.logwarn(f"Error reading/publishing frame: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
