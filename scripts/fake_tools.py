#!/usr/bin/env python3
import rospy
from robot_toolkit_msgs.srv import (
    motion_tools_srv, vision_tools_srv, misc_tools_srv, 
    tablet_service_srv, battery_service_srv, navigation_tools_srv, 
    audio_tools_srv, set_output_volume_srv
)

class NaoMockupNode:
    def __init__(self):
        rospy.init_node('nao_mockup_node', anonymous=True)
        
        # Set up mock services
        self.motion_tools = rospy.Service('/robot_toolkit/motion_tools_srv', motion_tools_srv, self.mock_response)
        self.vision_tools = rospy.Service('/robot_toolkit/vision_tools_srv', vision_tools_srv, self.mock_response)
        self.misc_tools = rospy.Service('/robot_toolkit/misc_tools_srv', misc_tools_srv, self.mock_response)
        self.navigation_tools = rospy.Service('/robot_toolkit/navigation_tools_srv', navigation_tools_srv, self.mock_response)
        self.audio_tools = rospy.Service('/robot_toolkit/audio_tools_srv', audio_tools_srv, self.mock_response)
        
        # Pytoolkit services
        self.tablet_show_web = rospy.Service('pytoolkit/ALTabletService/show_web_view_srv', tablet_service_srv, self.mock_response)
        self.tablet_show_image = rospy.Service('pytoolkit/ALTabletService/show_image_srv', tablet_service_srv, self.mock_response)
        self.get_battery_percentage = rospy.Service('pytoolkit/ALBatteryService/get_percentage', battery_service_srv, self.mock_response)
        self.get_output_volume = rospy.Service('pytoolkit/ALAudioDevice/get_output_volume_srv', battery_service_srv, self.mock_response)
        self.set_output_volume = rospy.Service('pytoolkit/ALAudioDevice/set_output_volume_srv', set_output_volume_srv, self.mock_response)

    def mock_response(self, req):
        rospy.loginfo(f"Received request: {req}")
        return "OK"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    mockup_node = NaoMockupNode()
    mockup_node.run()