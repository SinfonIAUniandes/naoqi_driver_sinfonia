#!/usr/bin/env python3
import rospy
import qi
from robot_toolkit_msgs.msg import speech_msg, speech_parameters_msg

class SpeechSubscriber:
    def __init__(self, session):
        self.session = session
        self.speech_service = session.service("ALTextToSpeech")
        self.animated_speech_service = session.service("ALAnimatedSpeech")
        self.language = "English"
        self.speech_service.setLanguage(self.language)
        
        rospy.init_node("speech_subscriber", anonymous=True)
        self.subscriber = rospy.Subscriber("/speech", speech_msg, self.speech_callback)
        rospy.loginfo("Speech Subscriber initialized.")

    def speech_callback(self, message):
        if message.language in ["Spanish", "English"]:
            if message.language != self.language:
                self.language = message.language
                self.speech_service.setLanguage(self.language)
            
            if message.animated:
                self.animated_speech_service.say(message.text)
            else:
                self.speech_service.say(message.text)
            
            rospy.loginfo(f"[Speech] Robot saying: {message.text}")
        else:
            apology = "Lo siento, no se hablar en ese idioma" if self.language == "Spanish" else "I am sorry, I don't know that language"
            self.speech_service.say(apology)

    def get_parameters(self):
        params = [
            self.speech_service.getParameter("pitchShift"),
            self.speech_service.getParameter("doubleVoice"),
            self.speech_service.getParameter("doubleVoiceLevel"),
            self.speech_service.getParameter("doubleVoiceTimeShift"),
            self.speech_service.getParameter("speed")
        ]
        return params

    def set_default_parameters(self):
        defaults = {
            "English": [1.170, 0.0, 0.0, 0.0, 100.0],
            "Spanish": [1.25, 0.0, 0.0, 0.0, 90.0]
        }
        for param, value in zip(["pitchShift", "doubleVoice", "doubleVoiceLevel", "doubleVoiceTimeShift", "speed"], defaults[self.language]):
            self.speech_service.setParameter(param, value)
        return self.get_parameters()

    def set_parameters(self, parameters):
        param_names = ["pitchShift", "doubleVoice", "doubleVoiceLevel", "doubleVoiceTimeShift", "speed"]
        for param, value in zip(param_names, parameters):
            self.speech_service.setParameter(param, value)
        return self.get_parameters()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    session = qi.Session()
    robot_ip = rospy.get_param("~robot_ip", "157.253.113.142")
    port = rospy.get_param("~port", 9559)
    try:
        session.connect(f"tcp://{robot_ip}:{port}")
        rospy.loginfo(f"Connected to NAOqi at {robot_ip}:{port}")
    except Exception as e:
        rospy.logerr(f"Failed to connect: {e}")
        exit(1)
    
    speech_subscriber = SpeechSubscriber(session)
    speech_subscriber.run()