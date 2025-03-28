#! /usr/bin/env python3
# -*- encoding: UTF-8 -*-

import qi
import time
import threading
import rospy
import rospkg
import math
import numpy as np
import argparse
import sys
from robot_toolkit_msgs.srv import (
    say_to_file_srv,
    say_to_file_srvResponse,
    set_words_threshold_srv,
    Tshirt_color_srv,
    tablet_service_srv,
    go_to_posture_srv,
    go_to_posture_srvResponse,
    tablet_service_srvResponse,
    go_to_posture_srvRequest,
    set_output_volume_srv,
    set_output_volume_srvResponse,
    set_security_distance_srv,
    set_security_distance_srvResponse,
    get_input_srv,
    set_speechrecognition_srv,
    point_at_srv,
    point_at_srvResponse,
    set_open_close_hand_srv,
    set_open_close_hand_srvResponse,
    move_head_srv,
    move_head_srvRequest,
    move_head_srvResponse,
    set_angle_srv,
    set_angle_srvResponse,
    get_segmentation3D_srv,
    get_segmentation3D_srvResponse,
    set_move_arms_enabled_srv,
    set_move_arms_enabled_srvResponse,
    navigate_to_srv,
    navigate_to_srvResponse,
    set_stiffnesses_srv,
    set_stiffnesses_srvResponse,
    battery_service_srv,
    speech_recognition_srv,
)
from robot_toolkit_msgs.msg import (
    text_to_speech_status_msg,
    speech_recognition_status_msg,
    speech_msg,
    set_angles_msg,
)
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist
import ConsoleFormatter


class PyToolkit:
    # -----------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------INIT--------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def __init__(self, session):

        # CONSTANTS
        self.PYTOOLKIT_FOLDER = rospkg.RosPack().get_path("naoqi_driver")
        # Crea una constante para guardar las id's conseguidas por el pytoolkit/perception
        self.id = 0
        # Publishers
        self.ALTextToSpeechStatusPublisher = rospy.Publisher(
            "/pytoolkit/ALTextToSpeech/status", text_to_speech_status_msg, queue_size=10
        )
        print(consoleFormatter.format("ALTextToSpeech/status topic is up!", "OKGREEN"))

        self.ALMotionFailedPublisher = rospy.Publisher(
            "/pytoolkit/ALMotion/failed", speech_recognition_status_msg, queue_size=10
        )
        print(consoleFormatter.format("ALMotion/failed topic is up!", "OKGREEN"))

        self.ALSpeechRecognitionStatusPublisher = rospy.Publisher(
            "/pytoolkit/ALSpeechRecognition/status",
            speech_recognition_status_msg,
            queue_size=10,
        )
        print(
            consoleFormatter.format(
                "ALSpeechRecognition/status topic is up!", "OKGREEN"
            )
        )

        self.ALSpeechRecognitionDetectedPublisher = rospy.Publisher(
            "/pytoolkit/ALSpeechRecognition/SpeechDetected",
            speech_recognition_status_msg,
            queue_size=10,
        )
        print(
            consoleFormatter.format(
                "ALSpeechRecognition/SpeechDetected topic is up!", "OKGREEN"
            )
        )

        self.ALSensorsPublisher = rospy.Publisher(
            "/pytoolkit/ALSensors/obstacles",
            speech_recognition_status_msg,
            queue_size=10,
        )
        print(consoleFormatter.format("ALSensors/obstacles topic is up!", "OKGREEN"))

        self.ALGetAnglesPublisher = rospy.Publisher(
            "/pytoolkit/ALMotion/get_angles", set_angles_msg, queue_size=10
        )
        print(consoleFormatter.format("ALMotion/get_angles topic is up!", "OKGREEN"))

        self.publish_angles = "None"

        # Subscriber
        self.ALMotionMovePublisher = rospy.Subscriber(
            "/pytoolkit/ALMotion/move", Twist, self.on_move
        )
        print(consoleFormatter.format("ALMotion/move subscriber is up!", "OKGREEN"))

        self.ALMemory = session.service("ALMemory")

        self.ALTextToSpeechStatusSubscriber = self.ALMemory.subscriber(
            "ALTextToSpeech/Status"
        )
        self.ALTextToSpeechStatusSubscriber.signal.connect(self.on_tts_status)

        self.ALSpeechRecognitionSubscriber = self.ALMemory.subscriber("WordRecognized")
        self.ALSpeechRecognitionSubscriber.signal.connect(
            self.on_speech_recognition_status
        )

        self.ALSpeechDetectedSubscriber = self.ALMemory.subscriber("SpeechDetected")
        self.ALSpeechDetectedSubscriber.signal.connect(
            self.on_speech_recognition_detected
        )

        self.ALMotionFailedSubscriber = self.ALMemory.subscriber("ALMotion/MoveFailed")
        self.ALMotionFailedSubscriber.signal.connect(self.on_move_failed)

        # Perception Subscriber
        self.ALPeoplePerceptionSubscriber = self.ALMemory.subscriber(
            "PeoplePerception/JustArrived"
        )
        self.ALPeoplePerceptionSubscriber.signal.connect(self.on_Perception_Tshirt)

        # Navigation Subscriber
        # self.ALSensorsRightSubscriber = self.ALMemory.subscriber("Device/SubDeviceList/Platform/InfraredSpot/Right/Sensor/Value")
        # self.ALSensorsLeftSubscriber = self.ALMemory.subscriber("Device/SubDeviceList/Platform/InfraredSpot/Left/Sensor/Value")
        # self.ALSensorsRightSubscriber.signal.connect(self.on_Perception_Tshirt)
        # self.ALSensorsLeftSubscriber.signal.connect(self.on_Perception_Tshirt)
        # self.ALSensorsRightSubscriber = self.ALMemory.subscriber("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value")
        # self.ALSensorsRightSubscriber.signal.connect(self.on_Perception_Tshirt)

        # Service Naoqi Clients
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAutonomousLife = session.service("ALAutonomousLife")
        self.ALBasicAwareness = session.service("ALBasicAwareness")
        self.ALMotion = session.service("ALMotion")
        self.ALNavigation = session.service("ALNavigation")
        self.ALRobotPosture = session.service("ALRobotPosture")
        self.ALSpeechRecognitionService = session.service("ALSpeechRecognition")
        self.ALPeoplePerception = session.service("ALPeoplePerception")
        self.ALPhotoCapture = session.service("ALPhotoCapture")
        self.ALTrackerService = session.service("ALTracker")
        self.ALBatteryService = session.service("ALBattery")
        self.ALAudioPlayer = session.service("ALAudioPlayer")
        self.ALTextToSpeech = session.service("ALTextToSpeech")
        self.ALSpeakingMovement = session.service("ALSpeakingMovement")
        self.ALAutonomousBlinking = session.service("ALAutonomousBlinking")
        self.ALServiceManager = session.service("ALServiceManager")

        # Service ROS Servers - ALAudioDevice
        self.audioDeviceSetOutputVolumeServer = rospy.Service(
            "pytoolkit/ALAudioDevice/set_output_volume_srv",
            set_output_volume_srv,
            self.callback_audio_device_set_output_volume_srv,
        )
        print(
            consoleFormatter.format(
                "ALAudioDevice/set_output_volume_srv on!", "OKGREEN"
            )
        )
        self.audioDeviceGetOutputVolumeServer = rospy.Service(
            "pytoolkit/ALAudioDevice/get_output_volume_srv",
            battery_service_srv,
            self.callback_audio_device_get_output_volume_srv,
        )
        print(
            consoleFormatter.format(
                "ALAudioDevice/get_output_volume_srv on!", "OKGREEN"
            )
        )

        self.shutUpServer = rospy.Service(
            "pytoolkit/ALTextToSpeech/shut_up_srv",
            battery_service_srv,
            self.callback_shut_up_srv,
        )
        print(consoleFormatter.format("ALTextToSpeech/shut_up_srv on!", "OKGREEN"))

        self.sayToFileServer = rospy.Service(
            "pytoolkit/ALTextToSpeech/say_to_file_srv",
            say_to_file_srv,
            self.callback_say_to_file_srv,
        )
        print(consoleFormatter.format("ALTextToSpeech/say_to_file_srv on!", "OKGREEN"))

        self.audioHearingServer = rospy.Service(
            "pytoolkit/ALSpeechRecognition/set_speechrecognition_srv",
            set_speechrecognition_srv,
            self.callback_set_speechrecognition_srv,
        )
        print(
            consoleFormatter.format(
                "ALAudioDevice/set_output_volume_srv on!", "OKGREEN"
            )
        )

        self.wordsServer = rospy.Service(
            "pytoolkit/ALSpeechRecognition/set_words_srv",
            set_words_threshold_srv,
            self.callback_set_words_srv,
        )
        print(
            consoleFormatter.format("ALSpeechRecognition/set_words_srv on!", "OKGREEN")
        )

        self.languageServer = rospy.Service(
            "pytoolkit/ALSpeechRecognition/set_hot_word_language_srv",
            tablet_service_srv,
            self.callback_set_hot_word_language_srv,
        )
        print(
            consoleFormatter.format(
                "ALSpeechRecognition/set_hot_word_language_srv on!", "OKGREEN"
            )
        )

        self.playSoundEffect = rospy.Service(
            "pytoolkit/ALAudioPlayer/play_sound_effect_srv",
            tablet_service_srv,
            self.callback_play_sound_effect_srv,
        )
        print(
            consoleFormatter.format(
                "ALAudioPlayer/play_sound_effect_srv on!", "OKGREEN"
            )
        )

        self.followFace = rospy.Service(
            "pytoolkit/ALTracker/start_follow_face",
            battery_service_srv,
            self.callback_start_follow_face_srv,
        )
        print(consoleFormatter.format("ALTracker/start_follow_face on!", "OKGREEN"))

        # Service ROS Servers - ALAutonomousLife
        self.autonomousSetStateServer = rospy.Service(
            "pytoolkit/ALAutonomousLife/set_state_srv",
            SetBool,
            self.callback_autonomous_set_state_srv,
        )
        print(consoleFormatter.format("ALAutonomousLife/set_state_srv on!", "OKGREEN"))
        self.toggleBlinkingServer = rospy.Service(
            "pytoolkit/ALAutonomousBlinking/toggle_blinking_srv",
            SetBool,
            self.callback_toggle_blinking_srv,
        )
        print(
            consoleFormatter.format(
                "ALAutonomousBlinking/toggle_blinking_srv on!", "OKGREEN"
            )
        )

        # Service ROS Servers - ALBasicAwareness
        self.awarenessSetAwarenessServer = rospy.Service(
            "pytoolkit/ALBasicAwareness/set_awareness_srv",
            SetBool,
            self.callback_awareness_set_awareness_srv,
        )
        print(consoleFormatter.format("Set_awareness_srv on!", "OKGREEN"))
        self.awarenessPauseAwarenessServer = rospy.Service(
            "pytoolkit/ALBasicAwareness/pause_awareness_srv",
            battery_service_srv,
            self.callback_awareness_pause_awareness_srv,
        )
        print(consoleFormatter.format("pause_awareness_srv on!", "OKGREEN"))
        self.awarenessResumeAwarenessServer = rospy.Service(
            "pytoolkit/ALBasicAwareness/resume_awareness_srv",
            battery_service_srv,
            self.callback_awareness_resume_awareness_srv,
        )
        print(consoleFormatter.format("resume_awareness_srv on!", "OKGREEN"))

        # Service ROS Servers - ALMotion

        self.motionSetSecurityArmsServer = rospy.Service(
            "pytoolkit/ALMotion/set_arms_security_srv",
            SetBool,
            self.callback_set_arms_security_srv,
        )
        print(consoleFormatter.format("ALMotion/set_arms_security_srv on!", "OKGREEN"))

        self.motionSetSecurityDistanceServer = rospy.Service(
            "pytoolkit/ALMotion/set_security_distance_srv",
            set_security_distance_srv,
            self.callback_motion_set_security_distance_srv,
        )
        print(consoleFormatter.format("Set_security_distance_srv on!", "OKGREEN"))

        self.motionSetTangentialSecurityDistanceServer = rospy.Service(
            "pytoolkit/ALMotion/set_tangential_security_distance_srv",
            set_security_distance_srv,
            self.callback_motion_set_tangential_security_distance_srv,
        )
        print(
            consoleFormatter.format(
                "set_tangential_security_distance_srv on!", "OKGREEN"
            )
        )

        self.motionSetOrthogonalSecurityDistanceServer = rospy.Service(
            "pytoolkit/ALMotion/set_orthogonal_security_distance_srv",
            set_security_distance_srv,
            self.callback_motion_set_orthogonal_security_distance_srv,
        )
        print(
            consoleFormatter.format(
                "set_orthogonal_security_distance_srv on!", "OKGREEN"
            )
        )

        self.motionSetOpenCloseHandServer = rospy.Service(
            "pytoolkit/ALMotion/set_open_close_hand_srv",
            set_open_close_hand_srv,
            self.callback_motion_set_open_close_hand_srv,
        )
        print(consoleFormatter.format("Set_open_close_hand_srv on!", "OKGREEN"))

        self.motionToggleBreathing = rospy.Service(
            "pytoolkit/ALMotion/toggle_breathing_srv",
            set_open_close_hand_srv,
            self.callback_toggle_breathing_srv,
        )
        print(consoleFormatter.format("toggle_breathing_srv on!", "OKGREEN"))

        # push test

        self.motionMoveHeadServer = rospy.Service(
            "pytoolkit/ALMotion/move_head_srv",
            move_head_srv,
            self.callback_motion_move_head_srv,
        )
        print(consoleFormatter.format("Move_head_srv on!", "OKGREEN"))

        self.motionSetAngleServer = rospy.Service(
            "pytoolkit/ALMotion/set_angle_srv",
            set_angle_srv,
            self.callback_motion_set_angle_srv,
        )
        print(consoleFormatter.format("set_angle_srv on!", "OKGREEN"))

        self.toggleGetAngleServer = rospy.Service(
            "pytoolkit/ALMotion/toggle_get_angle_srv",
            set_angle_srv,
            self.callback_toggle_get_angle_srv,
        )
        print(consoleFormatter.format("toggle_get_angle_srv on!", "OKGREEN"))

        self.motionSetMoveArmsEnabledServer = rospy.Service(
            "pytoolkit/ALMotion/set_move_arms_enabled_srv",
            set_move_arms_enabled_srv,
            self.callback_motion_set_move_arms_enabled_srv,
        )
        print(consoleFormatter.format("Set_move_arms_enabled_srv on!", "OKGREEN"))

        self.motionSetStiffnessesServer = rospy.Service(
            "pytoolkit/ALMotion/set_stiffnesses_srv",
            set_stiffnesses_srv,
            self.callback_set_stiffnesses_srv,
        )
        print(consoleFormatter.format("set_stiffnesses_srv on!", "OKGREEN"))

        self.motionToggleSmartStiffnessServer = rospy.Service(
            "pytoolkit/ALMotion/toggle_smart_stiffness_srv",
            SetBool,
            self.callback_toggle_smart_stiffness_srv,
        )
        print(consoleFormatter.format("toggle_smart_stiffness_srv on!", "OKGREEN"))

        self.playAudioStreamServer = rospy.Service(
            "pytoolkit/ALAudioPlayer/play_audio_stream_srv",
            set_stiffnesses_srv,
            self.callback_play_audio_stream_srv,
        )
        print(consoleFormatter.format("play_audio_stream_srv on!", "OKGREEN"))

        self.stopAudioServer = rospy.Service(
            "pytoolkit/ALAudioPlayer/stop_audio_stream_srv",
            battery_service_srv,
            self.callback_stop_audio_stream_srv,
        )
        print(consoleFormatter.format("stop_audio_stream_srv on!", "OKGREEN"))

        self.playDanceServer = rospy.Service(
            "pytoolkit/ALMotion/play_dance_srv",
            set_output_volume_srv,
            self.callback_play_dance_srv,
        )
        print(consoleFormatter.format("play_dance_srv on!", "OKGREEN"))

        # Service ROS Servers - ALNavigation
        self.navigationNavigateToServer = rospy.Service(
            "pytoolkit/ALMotion/move_relative_srv",
            navigate_to_srv,
            self.callback_move_relative_srv,
        )
        print(consoleFormatter.format("move_relative_srv on!", "OKGREEN"))
        self.navigationNavigateToServer = rospy.Service(
            "pytoolkit/ALNavigation/navigate_to_srv",
            navigate_to_srv,
            self.callback_navigation_navigate_to_srv,
        )
        print(consoleFormatter.format("Navigate_to_srv on!", "OKGREEN"))
        self.navigationStartExploringServer = rospy.Service(
            "pytoolkit/ALNavigation/start_exploring_srv",
            set_output_volume_srv,
            self.callback_start_exploring_srv,
        )
        print(consoleFormatter.format("start_exploring_srv on!", "OKGREEN"))
        self.navigationStopExploringServer = rospy.Service(
            "pytoolkit/ALNavigation/stop_exploring_srv",
            battery_service_srv,
            self.callback_stop_exploring_srv,
        )
        print(consoleFormatter.format("stop_exploring_srv on!", "OKGREEN"))

        # Service ROS Servers - ALRobotPosture
        self.postureGoToPostureServer = rospy.Service(
            "pytoolkit/ALRobotPosture/go_to_posture_srv",
            go_to_posture_srv,
            self.callback_posture_go_to_posture_srv,
        )
        print(consoleFormatter.format("Go_to_posture_srv on!", "OKGREEN"))

        # Service ROS Servers - ALBatteryService
        self.getBatteryPorcentage = rospy.Service(
            "pytoolkit/ALBatteryService/get_porcentage",
            battery_service_srv,
            self.callback_battery_get_porcentage_srv,
        )
        print(consoleFormatter.format("get_porcentage_srv on!", "OKGREEN"))


        # Service ROS Servers - ALTracker
        self.trackerPointAtServer = rospy.Service(
            "pytoolkit/ALTracker/point_at_srv", point_at_srv, self.callback_point_at_srv
        )
        print(consoleFormatter.format("Point_at_srv on!", "OKGREEN"))
        self.trackerPauseTrackerServer = rospy.Service(
            "pytoolkit/ALTracker/stop_tracker_srv",
            battery_service_srv,
            self.callback_stop_tracker_srv,
        )
        print(consoleFormatter.format("Stop_tracker_srv on!", "OKGREEN"))
        self.trackerStartTrackerServer = rospy.Service(
            "pytoolkit/ALTracker/start_tracker_srv",
            battery_service_srv,
            self.callback_start_tracker_srv,
        )
        print(consoleFormatter.format("Start_tracker_srv on!", "OKGREEN"))

        self.input = ""
        self.promise = qi.Promise()

        # Last coordinates send to the robot to move
        self.x = 0
        self.y = 0
        self.theta = 0

        # Probability threshold for publishing to the topic
        self.threshold = []
        self.words = []
        self.posture = "stand"
        # Variable for show_words service
        self.showing_words = False

        # Service ROS Servers - ALPerception
        # self.Server = rospy.Service('pytoolkit/ALPerception/Tshirt_color_srv', Tshirt_color_srv, self.callback_Tshirt_color__srv)
        # print(consoleFormatter.format('Tshirt_color__srv!', 'OKGREEN'))

    # -----------------------------------------------------------------------------------------------------------------------
    # ----------------------------------------------------SERVICES CALLBACKS-------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    # ----------------------------------------------------ALSpeechRecognition------------------------------------------------------

    def callback_audio_device_set_output_volume_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALAudioDevice/set_output_volume_srv", "WARNING"
            )
        )
        self.ALAudioDevice.setOutputVolume(req.volume)
        print(consoleFormatter.format("Volume set to " + str(req.volume), "OKGREEN"))
        return set_output_volume_srvResponse("OK")

    def callback_audio_device_get_output_volume_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALAudioDevice/get_output_volume_srv", "WARNING"
            )
        )
        self.ALAudioDevice.getOutputVolume()
        return str(self.ALAudioDevice.getOutputVolume())

    def callback_shut_up_srv(self, req):
        print(
            consoleFormatter.format("\nRequested ALTextToSpeech/shut_up_srv", "WARNING")
        )
        self.ALTextToSpeech.stopAll()
        return str("OK")

    def callback_say_to_file_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALTextToSpeech/say_to_file_srv", "WARNING"
            )
        )
        self.ALTextToSpeech.sayToFile(req.text, "/tmp/say_to_file.raw")
        with open("/tmp/say_to_file.raw", "rb") as f:
            audio_bytes = f.read()
        response = say_to_file_srvResponse()
        response.data = np.frombuffer(audio_bytes, dtype=np.uint8).tolist()
        return response

    # ----------------------------------------------------ALAudioDevice------------------------------------------------------

    def callback_set_words_srv(self, req):
        if len(req.words) > 0:
            self.ALSpeechRecognitionService.pause(True)
            self.ALSpeechRecognitionService.removeAllContext()
            self.ALSpeechRecognitionService.setVocabulary(req.words, False)
            self.words = req.words
            self.threshold = req.threshold
            self.ALSpeechRecognitionService.pause(False)
        return "OK"

    def callback_set_hot_word_language_srv(self, req):
        self.ALSpeechRecognitionService.pause(True)
        self.ALSpeechRecognitionService.setLanguage(req.url)
        self.ALSpeechRecognitionService.pause(False)
        return tablet_service_srvResponse("OK")

    def callback_set_speechrecognition_srv(self, req):
        self.ALSpeechRecognitionService.pause(True)
        if req.subscribe:
            self.ALSpeechRecognitionService.subscribe("isHearing")
            self.ALSpeechRecognitionService.pause(False)
        else:
            self.ALSpeechRecognitionService.unsubscribe("isHearing")
            self.ALSpeechRecognitionService.pause(True)
        self.ALSpeechRecognitionService.setAudioExpression(req.noise)
        self.ALSpeechRecognitionService.setVisualExpression(req.eyes)
        return "OK"

    def callback_play_sound_effect_srv(self, req):
        self.ALAudioPlayer.playSoundSetFile(req.url)
        return tablet_service_srvResponse("OK")

    def callback_play_audio_stream_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALAudioPlayer/play_audio_stream_srv", "WARNING"
            )
        )
        self.ALAudioPlayer.playWebStream(req.names, req.stiffnesses, 0)
        print(consoleFormatter.format("Stream played!", "OKGREEN"))
        return set_stiffnesses_srvResponse("OK")

    def callback_stop_audio_stream_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALAudioPlayer/stop_audio_stream_srv", "WARNING"
            )
        )
        self.ALAudioPlayer.stopAll()
        print(consoleFormatter.format("Stream stopped!", "OKGREEN"))
        return str("OK")

    def callback_play_dance_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/play_dance_srv", "WARNING"))
        self.ALRobotPosture.goToPosture("Stand", 0.5)
        if req.volume == 1:
            dance_arcadia.dance(self.ALMotion, self.ALAudioPlayer)
        elif req.volume == 2:
            dance_hands.dance(self.ALMotion)
        elif req.volume == 3:
            dance_asereje.dance(self.ALMotion, self.ALAudioPlayer)
        self.ALAudioPlayer.stopAll()
        return str("OK")

    # ----------------------------------------------------ALAutonomousLife------------------------------------------------

    def callback_autonomous_set_state_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALAutonomousLife/set_state_srv", "WARNING"
            )
        )
        self.ALAutonomousLife.setAutonomousAbilityEnabled("All", req.data)
        if req.data:
            self.ALAutonomousLife.setState("interactive")
            print(consoleFormatter.format("Autonomous life is on!", "OKGREEN"))
        else:
            self.ALAutonomousLife.setState("disabled")
            self.ALAutonomousLife.stopAll()
            self.callback_posture_go_to_posture_srv(go_to_posture_srvRequest("stand"))
            print(consoleFormatter.format("Autonomous life is off!", "OKGREEN"))
        return SetBoolResponse(True, "OK")

    # ----------------------------------------------------ALBasicAwareness------------------------------------------------
    # Este servicio es para poner el estado del awareness
    def callback_awareness_set_awareness_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALBasicAwareness/set_awareness_srv", "WARNING"
            )
        )
        self.callback_motion_move_head_srv(move_head_srvRequest("default"))
        self.ALBasicAwareness.setEnabled(req.data)
        if req.data:
            print(consoleFormatter.format("Awareness is on!", "OKGREEN"))
        else:
            print(consoleFormatter.format("Awareness is off!", "OKGREEN"))
        return SetBoolResponse(True, "OK")

    # Este es para pausar el awareness
    def callback_awareness_pause_awareness_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALBasicAwareness/pause_awareness_srv", "WARNING"
            )
        )
        self.callback_motion_move_head_srv(move_head_srvRequest("default"))
        self.ALBasicAwareness.startAwareness()
        self.ALBasicAwareness.resumeAwareness()
        self.ALBasicAwareness.pauseAwareness()
        self.ALBasicAwareness.stopAwareness()
        print(consoleFormatter.format("Awareness is paused!", "OKGREEN"))
        return "OK"

    def callback_toggle_blinking_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALAutonomousBlinking/toggle_blinking_srv", "WARNING"
            )
        )
        self.ALAutonomousBlinking.setEnabled(req.data)
        return SetBoolResponse(True, "OK")

    # Este es para reanudar el awarenes
    def callback_awareness_resume_awareness_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALBasicAwareness/start_awareness_srv", "WARNING"
            )
        )
        self.callback_motion_move_head_srv(move_head_srvRequest("default"))
        self.ALBasicAwareness.stopAwareness()
        self.ALBasicAwareness.startAwareness()
        self.ALBasicAwareness.pauseAwareness()
        self.ALBasicAwareness.resumeAwareness()
        print(consoleFormatter.format("Awareness is resumed!", "OKGREEN"))
        return "OK"

    def callback_toggle_breathing_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/toggle_breathing_srv", "WARNING"
            )
        )
        if req.state == "True" :
            
            if self.posture == "stand":
                self.ALMotion.setBreathEnabled(req.hand, True)
            else:
                if self.posture == "rest":
                    # DO NOT ALLOW BREATHING WHEN RESTING
                    pass
                elif self.posture == "sit" or self.posture == "sitrelax":
                    # If the robot is sitting, let him move his upper body, but not his legs
                    if req.hand != "Body" and req.hand != "Legs":
                        self.ALMotion.setBreathEnabled(req.hand, True)
                    
        if req.state == "False":
            self.ALMotion.setBreathEnabled(req.hand, False)
        return set_open_close_hand_srvResponse("OK")

    # ----------------------------------------------------ALMotion------------------------------------------------

    def callback_motion_set_security_distance_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/set_security_distance_srv", "WARNING"
            )
        )
        self.ALMotion.setOrthogonalSecurityDistance(req.distance)
        self.ALMotion.setTangentialSecurityDistance(req.distance)
        self.ALMotion.setCollisionProtectionEnabled("Arms", False)
        self.ALMotion.setExternalCollisionProtectionEnabled("All", False)
        print(
            consoleFormatter.format(
                "Security distance was set to " + str(req.distance) + " m", "OKGREEN"
            )
        )
        return set_security_distance_srvResponse("OK")

    def callback_set_arms_security_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/set_arms_security_srv", "WARNING"
            )
        )
        self.ALMotion.setCollisionProtectionEnabled("Arms", req.data)
        self.ALMotion.setExternalCollisionProtectionEnabled("Arms", req.data)
        return SetBoolResponse(True, "OK")

    def callback_motion_set_tangential_security_distance_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/set_tangential_security_distance_srv", "WARNING"
            )
        )
        self.ALMotion.setTangentialSecurityDistance(req.distance)
        return set_security_distance_srvResponse("OK")

    def callback_motion_set_orthogonal_security_distance_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/set_orthogonal_security_distance_srv", "WARNING"
            )
        )
        self.ALMotion.setOrthogonalSecurityDistance(req.distance)
        return set_security_distance_srvResponse("OK")

    def callback_motion_enable_security_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/enable_security_srv", "WARNING"
            )
        )
        self.ALMotion.setOrthogonalSecurityDistance(0.4)
        self.ALMotion.setTangentialSecurityDistance(0.1)
        self.ALMotion.setCollisionProtectionEnabled("Arms", True)
        self.ALMotion.setExternalCollisionProtectionEnabled("All", True)
        print(consoleFormatter.format("Security distance was enabled", "OKGREEN"))
        return str("OK")

    def callback_motion_set_open_close_hand_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/open_close_hand_srv", "WARNING"
            )
        )
        if req.hand == "left" or req.hand == "both":
            if req.state == "open":
                self.ALMotion.setAngles("LHand", 1.0, 0.2)
                print(consoleFormatter.format("Left hand is open!", "OKGREEN"))
            elif req.state == "close":
                self.ALMotion.setAngles("LHand", 0.0, 0.2)
                print(consoleFormatter.format("Left hand is closed!", "OKGREEN"))
        if req.hand == "right" or req.hand == "both":
            if req.state == "open":
                self.ALMotion.setAngles("RHand", 1.0, 0.2)
                print(consoleFormatter.format("Right hand is open!", "OKGREEN"))
            elif req.state == "close":
                self.ALMotion.setAngles("RHand", 0.0, 0.2)
                print(consoleFormatter.format("Right hand is closed!", "OKGREEN"))
        return set_open_close_hand_srvResponse("OK")

    def callback_motion_move_head_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/move_head_srv", "WARNING"))
        if req.state == "up":
            self.ALMotion.setAngles(["HeadPitch", "HeadYaw"], [-0.4, 0.0], 0.15)
            print(consoleFormatter.format("Head is up!", "OKGREEN"))
        elif req.state == "down":
            self.ALMotion.setAngles(["HeadPitch", "HeadYaw"], [0.46, 0.0], 0.2)
            print(consoleFormatter.format("Head is down!", "OKGREEN"))
        elif req.state == "default":
            self.ALMotion.setAngles(["HeadPitch", "HeadYaw"], [0.0, 0.0], 0.2)
            print(consoleFormatter.format("Head is in default position!", "OKGREEN"))
        return move_head_srvResponse("OK")

    def callback_motion_set_angle_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/set_angle_srv", "WARNING"))
        self.ALMotion.setAngles(tuple(req.name), tuple(req.angle), req.speed)
        print(str(req.name) + str(req.angle) + str(req.speed))
        print(consoleFormatter.format("Angles set!", "OKGREEN"))
        return set_angle_srvResponse("OK")

    def callback_toggle_get_angle_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/turn_get_angle_srv", "WARNING"
            )
        )
        self.publish_angles = req.name
        print(req)
        publish_angles_thread = threading.Thread(target=self.get_angles_thread)
        publish_angles_thread.start()
        print(
            consoleFormatter.format(
                "Started publisihing /pytoolkit/ALMotion/get_angles", "OKGREEN"
            )
        )
        return set_angle_srvResponse("OK")

    def callback_motion_set_move_arms_enabled_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/set_move_arms_enabled_srv", "WARNING"
            )
        )
        self.ALMotion.setMoveArmsEnabled(req.LArm, req.RArm)
        while True:
            LArmEnabled = self.ALMotion.getMoveArmsEnabled("LArm")
            RArmEnabled = self.ALMotion.getMoveArmsEnabled("RArm")
            if LArmEnabled != req.LArm or RArmEnabled != req.RArm:
                self.ALMotion.setMoveArmsEnabled(req.LArm, req.RArm)
            else:
                break
        if req.LArm:
            print(consoleFormatter.format("LArm movement has been enabled!", "OKGREEN"))
        else:
            print(
                consoleFormatter.format("LArm movement has been disabled!", "OKGREEN")
            )
        if req.RArm:
            print(consoleFormatter.format("RArm movement has been enabled!", "OKGREEN"))
        else:
            print(
                consoleFormatter.format("RArm movement has been disabled!", "OKGREEN")
            )
        return set_move_arms_enabled_srvResponse("OK")

    def callback_set_stiffnesses_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/set_stiffnesses_srv", "WARNING"
            )
        )
        self.ALMotion.setStiffnesses(req.names, req.stiffnesses)
        print(consoleFormatter.format("Stiffness set!", "OKGREEN"))
        return set_stiffnesses_srvResponse("OK")

    def callback_toggle_smart_stiffness_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALMotion/toggle_smart_stiffness_srv", "WARNING"
            )
        )
        self.ALMotion.setSmartStiffnessEnabled(req.data)
        return SetBoolResponse(True, "OK")

    # ----------------------------------------------------ALNavigation-------------------------------------------------

    def callback_move_relative_srv(self, req):
        print(
            consoleFormatter.format("\nRequested ALMotion/move_relative_srv", "WARNING")
        )
        self.ALMotion.moveTo(req.x_coordinate, req.y_coordinate, 0)
        print(
            consoleFormatter.format(
                "Robot is moving to the given coordinates!", "OKGREEN"
            )
        )
        return navigate_to_srvResponse("OK")

    def callback_navigation_navigate_to_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALNavigation/navigate_to_srv", "WARNING"
            )
        )
        self.ALNavigation.navigateTo(req.x_coordinate, req.y_coordinate)
        print(
            consoleFormatter.format(
                "Robot is navigating to the given coordinates!", "OKGREEN"
            )
        )
        return navigate_to_srvResponse("OK")

    def callback_start_exploring_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALNavigation/start_exploring_srv", "WARNING"
            )
        )
        try:
            print(
                consoleFormatter.format(
                    "Robot is exploring the surrounding " + str(req.volume) + " meters",
                    "OKGREEN",
                )
            )
            self.ALNavigation.explore(req.volume)
        except RuntimeError:
            print(
                consoleFormatter.format(
                    "Robot ended exploring the surrounding "
                    + str(req.volume)
                    + " meters",
                    "OKGREEN",
                )
            )
        return set_output_volume_srvResponse("OK")

    def callback_stop_exploring_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALNavigation/stop_exploring_srv", "WARNING"
            )
        )
        self.ALNavigation.stopExploration()
        print(
            consoleFormatter.format(
                "Robot has stopped exploring the surrounding meters", "OKGREEN"
            )
        )
        return str("OK")

    # ----------------------------------------------------ALRobotPosture------------------------------------------------

    def callback_posture_go_to_posture_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALRobotPosture/go_to_posture_srv", "WARNING"
            )
        )
        if req.posture == "stand":
            self.ALMotion.setBreathEnabled("All", False)
            self.ALRobotPosture.goToPosture("Stand", 0.5)
            self.ALMotion.wakeUp()
            print(consoleFormatter.format("Robot is in default position!", "OKGREEN"))
            self.posture = "stand"
        elif req.posture == "rest":
            self.ALMotion.setBreathEnabled("All", False)
            self.ALRobotPosture.goToPosture("Crouch", 0.5)
            self.ALMotion.rest()
            print(consoleFormatter.format("Robot is in rest position!", "OKGREEN"))
            self.posture = "rest"
        elif req.posture == "sit":
            self.ALMotion.setBreathEnabled("All", False)
            self.ALRobotPosture.goToPosture("Sit", 0.5)
            print(consoleFormatter.format("Robot is in sitting position!", "OKGREEN"))
            self.posture = "sit"
        elif req.posture == "sitrelax":
            self.ALMotion.setBreathEnabled("All", False)
            self.ALRobotPosture.goToPosture("SitRelax", 0.5)
            print(consoleFormatter.format("Robot is in sitting position!", "OKGREEN"))
            self.posture = "sitrelax"
        return go_to_posture_srvResponse("OK")

    # ----------------------------------------------------ALRobotPosture------------------------------------------------

    def callback_segmentation3D_get_segmentation3D_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALSegmentation3D/get_segmentation3D_srv", "WARNING"
            )
        )
        response = get_segmentation3D_srvResponse()
        coordinates = list(self.ALSegmentation3D.getTopOfBlob(-1, 0, False))
        response.coordinates = coordinates
        return response

    # ----------------------------------------------------ALPerception------------------------------------------------

    def callback_Tshirt_color__srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALPerception/Tshirt_color_srv", "WARNING"
            )
        )
        id = self.id
        color = self.ALMemory.getData("PeoplePerception/Person/" + id + "/ShirtColor")
        return color

    # ----------------------------------------------------ALBatteryService------------------------------------------------

    def callback_battery_get_porcentage_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALBatteryService/get_porcentage_srv", "WARNING"
            )
        )
        return str(self.ALBatteryService.getBatteryCharge())

    # ----------------------------------------------------ALTrackerService------------------------------------------------

    def callback_point_at_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTracker/point_at_srv", "WARNING"))
        self.ALTrackerService.pointAt(
            req.effector_name, [req.x, req.y, req.z], req.frame, req.speed
        )
        print(consoleFormatter.format("Pointing at!", "OKGREEN"))
        return point_at_srvResponse("OK")

    # Este es para parar el tracker
    def callback_stop_tracker_srv(self, req):
        print(
            consoleFormatter.format("\nRequested ALTracker/stop_tracker_srv", "WARNING")
        )
        self.callback_motion_move_head_srv(move_head_srvRequest("default"))
        self.ALTrackerService.setEffector("None")
        self.ALTrackerService.stopTracker()
        self.ALTrackerService.unregisterAllTargets()
        print(consoleFormatter.format("Tracker has stopped!", "OKGREEN"))
        return "OK"

    # Este es para reiniciar el tracker
    def callback_start_tracker_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALTracker/start_tracker_srv", "WARNING"
            )
        )
        self.callback_motion_move_head_srv(move_head_srvRequest("default"))
        self.ALTrackerService.setEffector("None")
        targetName = "Face"
        faceWidth = 0.1
        self.ALTrackerService.registerTarget(targetName, faceWidth)
        self.ALTrackerService.setRelativePosition([0.3, 0.0, 0.0,
                                                   0.1, 0.1, 0.3])
        self.ALTrackerService.setMode("Head")
        self.ALTrackerService.track(targetName)
        self.ALTrackerService.initialize()
        print(consoleFormatter.format("Tracker has started!", "OKGREEN"))
        return "OK"

    def callback_start_follow_face_srv(self, req):
        print(
            consoleFormatter.format(
                "\nRequested ALTracker/start_follow_face_srv", "WARNING"
            )
        )
        self.callback_motion_move_head_srv(move_head_srvRequest("default"))
        self.ALTrackerService.setMaximumDistanceDetection(3.5)
        self.ALTrackerService.initialize()
        self.ALTrackerService.setMode("Move")
        self.ALTrackerService.registerTarget("Face", 0.2)
        self.ALTrackerService.track("Face")
        print(consoleFormatter.format("Follow Face has started!", "OKGREEN"))
        return "OK"

    # -----------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------EVENTS CALLBACKS--------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def on_tts_status(self, value):
        idOfConcernedTask, status = value
        self.ALTextToSpeechStatusPublisher.publish(
            text_to_speech_status_msg(idOfConcernedTask, status)
        )

    def on_move_failed(self, value):
        self.ALMotionFailedPublisher.publish(speech_recognition_status_msg(value[0]))

    def on_speech_recognition_status(self, value):
        word = value[0]
        number = value[1]
        try:
            index = self.words.index(word)
            if number > self.threshold[index]:
                self.ALSpeechRecognitionStatusPublisher.publish(
                    speech_recognition_status_msg(word)
                )
        except ValueError:
            print("word not in list")

    def on_speech_recognition_detected(self, value):
        if value == 0:
            self.ALSpeechRecognitionDetectedPublisher.publish(
                speech_recognition_status_msg("stopped")
            )
        if value == 1:
            self.ALSpeechRecognitionDetectedPublisher.publish(
                speech_recognition_status_msg("started")
            )

    def get_angles_thread(self):
        while self.publish_angles != "None":
            try:
                angle = self.ALMotion.getAngles(self.publish_angles, False)
                msg = set_angles_msg()
                msg.names = self.publish_angles
                msg.angles = angle
                msg.fraction_max_speed = []
                self.ALGetAnglesPublisher.publish(msg)
                rospy.sleep(0.1)
            except:
                pass

    def on_Perception_Tshirt(self, id):
        self.id = id

    def on_ObstacleDetection(self, value):
        print(value)
        # self.ALSpeechRecognitionStatusPublisher.publish(speech_recognition_status_msg(value))

    def on_move(self, msg):
        print(consoleFormatter.format("\nRequested ALMotion/move", "WARNING"))
        if (msg.linear.x == 0) and (msg.linear.y == 0) and (msg.angular.z == 0):
            print(consoleFormatter.format("\nStopping Movement", "WARNING"))
            self.ALMotion.stopMove()
        else:
            self.x = msg.linear.x
            self.y = msg.linear.y
            self.theta = msg.angular.z
            self.ALMotion.move(msg.linear.x, msg.linear.y, msg.angular.z)


if __name__ == "__main__":
    consoleFormatter = ConsoleFormatter.ConsoleFormatter()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ip",
        type=str,
        default="157.253.113.142",
        help="Robot IP address. On RObot or Local Naoqi: use '127.0.0.1'.",
    )
    parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
    args, unknown = parser.parse_known_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print(
            "Can't connect to Naoqi at ip \""
            + args.ip
            + '" on port'
            + str(args.port)
            + ".\n"
            "Please check your script arguments. Run with -h option for help."
        )
        sys.exit(1)
    pytoolkit = PyToolkit(session)
    rospy.init_node("pytoolkit")
    try:
        try:
            pytoolkit.ALServiceManager.stop("AppLauncher")
        except:
            pass
        pytoolkit.ALAutonomousBlinking.setEnabled(True)
        if pytoolkit.ALAutonomousLife.getState() != "disabled":
            pytoolkit.ALAutonomousLife.setState("disabled")
            # En teoria evita que el robot se le dañe el brazo
            rospy.sleep(2)
            pytoolkit.ALRobotPosture.goToPosture("Stand", 0.5)
            print(consoleFormatter.format("Robot is in default position!", "OKGREEN"))
        if pytoolkit.ALBasicAwareness.isEnabled():
            pytoolkit.ALBasicAwareness.setEnabled(False)
        pytoolkit.ALTrackerService.stopTracker()
        pytoolkit.ALSpeakingMovement.setEnabled(True)
        time.sleep(1)
        print(
            consoleFormatter.format(
                " \n----------------------------------------------------------",
                "OKGREEN",
            )
        )
        print(
            consoleFormatter.format(
                " --------- PyToolkit node successfully initialized --------- ",
                "OKGREEN",
            )
        )
        print(
            consoleFormatter.format(
                " ----------------------------------------------------------\n",
                "OKGREEN",
            )
        )
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
