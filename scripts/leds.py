#! /usr/bin/env python3
# -*- encoding: UTF-8 -*-

import rospy
import qi
import sys
from robot_toolkit_msgs.msg import leds_parameters_msg

class LedsSubscriber:
    def __init__(self, session, topic):
        self.session = session
        self.topic = topic
        self.leds_service = self.session.service("ALLeds")
        self.is_initialized = False
    
    def start(self):
        self.subscriber = rospy.Subscriber(self.topic, leds_parameters_msg, self.leds_callback)
        self.is_initialized = True

    def leds_callback(self, message):
        red = message.red / 255.0
        green = message.green / 255.0
        blue = message.blue / 255.0
        
        try:
            if "Ear" in message.name:
                self.leds_service.fadeRGB(message.name, 0, 0, blue, message.time)
            else:
                self.leds_service.fadeRGB(message.name, red, green, blue, message.time)
            rospy.loginfo(f"Set LED {message.name} to ({red}, {green}, {blue}) over {message.time} seconds")
        except Exception as e:
            rospy.logerr(f"[{rospy.Time.now().to_sec()}] Error: Could not set LED {message.name}. {str(e)}")

    def shutdown(self):
        self.subscriber.unregister()
        self.is_initialized = False

    def get_parameters(self):
        return []

    def set_default_parameters(self):
        return self.get_parameters()

    def set_parameters(self, parameters):
        return self.get_parameters()

if __name__ == "__main__":
    rospy.init_node("leds_subscriber")
    
    robot_ip = rospy.get_param("~robot_ip", "127.0.0.1")
    port = rospy.get_param("~port", 9559)

    session = qi.Session()
    try:
        session.connect(f"tcp://{robot_ip}:{port}")
        rospy.loginfo(f"Connected to NAOqi at {robot_ip}:{port}")
    except RuntimeError:
        rospy.logerr(f"Can't connect to Naoqi at {robot_ip}:{port}")
        sys.exit(1)
    
    leds_subscriber = LedsSubscriber(session, "/leds")
    leds_subscriber.start()
    
    rospy.loginfo("LedsSubscriber node initialized.")
    rospy.spin()
