#! /usr/bin/env python3
# -*- encoding: UTF-8 -*-

import qi
import rospy
from sensor_msgs.msg import Range
import sys

def get_sonar_data(memory_service):
    """Retrieve sonar data from ALMemory."""
    left_value = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
    right_value = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")
    return left_value, right_value

def publish_sonar_data(session):
    """Initialize ROS node and publish sonar data."""
    rospy.init_node("nao_sonar_publisher", anonymous=True)
    left_pub = rospy.Publisher("/nao/sonar/left", Range, queue_size=10)
    right_pub = rospy.Publisher("/nao/sonar/right", Range, queue_size=10)
    
    memory_service = session.service("ALMemory")
    sonar_service = session.service("ALSonar")
    
    # Start sonars
    sonar_service.subscribe("nao_sonar_publisher")
    rospy.loginfo("Subscribed to NAO sonars.")
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        left_value, right_value = get_sonar_data(memory_service)
        
        left_msg = Range()
        left_msg.header.stamp = rospy.Time.now()
        left_msg.header.frame_id = "LSonar_frame"
        left_msg.radiation_type = Range.ULTRASOUND
        left_msg.field_of_view = 0.523598776  # Approximate field of view in radians
        left_msg.min_range = 0.25  # Minimum detection range
        left_msg.max_range = 2.55  # Maximum detection range
        left_msg.range = left_value if 0.25 <= left_value <= 2.55 else 2.55
        
        right_msg = Range()
        right_msg.header.stamp = rospy.Time.now()
        right_msg.header.frame_id = "RSonar_frame"
        right_msg.radiation_type = Range.ULTRASOUND
        right_msg.field_of_view = 0.523598776
        right_msg.min_range = 0.25
        right_msg.max_range = 2.55
        right_msg.range = right_value if 0.25 <= right_value <= 2.55 else 2.55
        
        left_pub.publish(left_msg)
        right_pub.publish(right_msg)
        
        rate.sleep()
    
    sonar_service.unsubscribe("nao_sonar_publisher")
    rospy.loginfo("Unsubscribed from NAO sonars.")

if __name__ == "__main__":
    robot_ip = rospy.get_param("~robot_ip", "157.253.113.142")
    port = rospy.get_param("~port", 9559)
    
    session = qi.Session()
    try:
        session.connect(f"tcp://{robot_ip}:{port}")
    except RuntimeError:
        rospy.logerr(f"Can't connect to Naoqi at {robot_ip}:{port}")
        sys.exit(1)
    
    publish_sonar_data(session)