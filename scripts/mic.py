#!/usr/bin/env python3
import sys
import numpy as np
import qi
import rospy
from naoqi_bridge_msgs.msg import AudioBuffer
import time

class AudioModule(object):
    def __init__(self, name, session):
        self.name = name
        self.session = session
        self.mic_sample_rate = 16000  # Set correct sample rate
        self.pub = rospy.Publisher('/mic', AudioBuffer, queue_size=10)
        self.audioProxy = session.service("ALAudioDevice")

        # Wait until after service registration
        rospy.loginfo("Audio module initialized, waiting to register...")
    
    def start(self):
        # Register the service first
        self.session.registerService("ROSDriverAudio", self)

        # Now set client preferences
        self.audioProxy.setClientPreferences("ROSDriverAudio", 16000, 3, 0)

        # Subscribe to ALAudioDevice
        self.audioProxy.subscribe("ROSDriverAudio")
        rospy.loginfo("Subscribed to ALAudioDevice.")

    def processRemote(self, nbOfChannels, nbOfSamples, timeStamp, buffer):
        msg = AudioBuffer()
        msg.header.stamp = rospy.Time.now()
        msg.frequency = self.mic_sample_rate

        try:
            audioData = np.frombuffer(buffer, dtype=np.int16)
        except Exception as e:
            rospy.logerr("Failed to convert audio buffer: %s", str(e))
            return

        msg.data = audioData.tolist()
        self.pub.publish(msg)

    def shutdown(self):
        try:
            self.audioProxy.unsubscribe("ROSDriverAudio")
            rospy.loginfo("Audio module unsubscribed.")
        except Exception as e:
            rospy.logwarn("Error while unsubscribing: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('naoqi_audio_publisher')
    session = qi.Session()
    try:
        session.listen("tcp://0.0.0.0:0")
    except RuntimeError as e:
        rospy.logwarn("Failed to set listen URL: %s", str(e))

    robot_ip = rospy.get_param("~robot_ip", "157.253.113.142")
    port = rospy.get_param("~port", 9559)
    
    try:
        session.connect(f"tcp://{robot_ip}:{port}")
        rospy.loginfo(f"Connected to NAOqi at {robot_ip}:{port}")
    except Exception as e:
        rospy.logerr("Failed to connect: %s", str(e))
        sys.exit(1)

    audio_module = AudioModule("ROSDriverAudio", session)
    audio_module.start()  # Call start after creation

    rospy.loginfo("Audio module running.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")
    finally:
        audio_module.shutdown()
