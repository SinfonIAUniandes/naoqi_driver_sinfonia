#! /usr/bin/env python3
# -*- encoding: UTF-8 -*-

import rospy
import qi
import sys
from robot_toolkit_msgs.msg import animation_msg

class AnimationSubscriber:
    def __init__(self, session, topic):
        self.session = session
        self.topic = topic
        self.behavior_manager = self.session.service("ALBehaviorManager")
        self.is_initialized = False
    
    def start(self):
        self.subscriber = rospy.Subscriber(self.topic, animation_msg, self.animation_callback)
        self.is_initialized = True

    def animation_callback(self, message):
        if message.family in ["animations", "animations_sinfonia"]:
            animation_name = f"animations/{message.animation_name}" if message.family == "animations" \
                else f"animations_sinfonia/animations/{message.animation_name}"

            installed_behaviors = self.behavior_manager.getInstalledBehaviors()
            
            if animation_name in installed_behaviors:
                rospy.loginfo(f"Starting animation: {animation_name}")
                self.behavior_manager.startBehavior(animation_name)
            else:
                rospy.logerr(f"[{rospy.Time.now().to_sec()}] Error: Animation does not exist: {animation_name}")
        else:
            rospy.logerr(f"[{rospy.Time.now().to_sec()}] Error: Unknown animation family")

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
    rospy.init_node("animation_subscriber")
    
    robot_ip = rospy.get_param("~robot_ip","127.0.0.1")
    port = rospy.get_param("~port", 9559)

    session = qi.Session()
    try:
        session.connect(f"tcp://{robot_ip}:{port}")
        rospy.loginfo(f"Connected to NAOqi at {robot_ip}:{port}")
    except RuntimeError:
        rospy.logerr(f"Can't connect to Naoqi at {robot_ip}:{port}")
        sys.exit(1)
    
    animation_subscriber = AnimationSubscriber(session, "animations")
    animation_subscriber.start()
    
    rospy.loginfo("AnimationSubscriber node initialized.")
    rospy.spin()
