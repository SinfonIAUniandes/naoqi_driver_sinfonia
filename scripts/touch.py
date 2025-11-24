#!/usr/bin/env python3
import qi
import functools
import sys
import rospy
from robot_toolkit_msgs.msg import touch_msg

# Mapping NAO sensor names to ROS-friendly names
touch_sensors = {
    "Head/Touch/Front": "head_front",
    "Head/Touch/Middle": "head_middle",
    "Head/Touch/Rear": "head_rear",
    "RHand/Touch/Back": "hand_right_back",
    "RHand/Touch/Left": "hand_right_left",
    "RHand/Touch/Right": "hand_right_right",
    "LHand/Touch/Back": "hand_left_back",
    "LHand/Touch/Left": "hand_left_left",
    "LHand/Touch/Right": "hand_left_right",
    "LFoot/Bumper/Left": "foot_left_bumper",
    "LFoot/Bumper/Right": "foot_left_bumper_right",
    "RFoot/Bumper/Left": "foot_right_bumper_left",
    "RFoot/Bumper/Right": "foot_right_bumper",
}

class ReactToTouch(object):
    """React to touch events and publish them as ROS messages."""
    
    def __init__(self, app):
        super(ReactToTouch, self).__init__()

        # Start NAOqi session
        app.start()
        session = app.session
        self.memory_service = session.service("ALMemory")

        # ROS node initialization moved to main so get_param works correctly
        self.publisher = rospy.Publisher("/touch", touch_msg, queue_size=10)

        # Subscribe to NAOqi "TouchChanged" event
        self.touch = self.memory_service.subscriber("TouchChanged")
        self.id = self.touch.signal.connect(functools.partial(self.onTouched, "TouchChanged"))
    
    def onTouched(self, strVarName, value):
        """Callback function triggered on touch event."""
        rospy.loginfo(f"Touch event detected: {value}")

        for sensor_data in value:
            sensor_name, is_touched = sensor_data[0], sensor_data[1]

            # Convert NAO sensor name to ROS-friendly name
            if sensor_name in touch_sensors:
                formatted_name = touch_sensors[sensor_name]
                msg = touch_msg()
                msg.name = formatted_name
                msg.state = bool(is_touched)  # Convert touch state to boolean
                self.publisher.publish(msg)  # Publish to ROS topic

                rospy.loginfo(f"Published {formatted_name}: {msg.state}")

    def run(self):
        """Keep ROS node alive."""
        rospy.spin()


if __name__ == "__main__":
    # Initialize ROS node before reading parameters so rospy.get_param("~robot_ip", ...) reads launch args
    rospy.init_node("naoqi_touch_publisher", anonymous=True)

    try:
        # Initialize NAOqi framework
        robot_ip = rospy.get_param("~robot_ip", "127.0.0.1")
        port = rospy.get_param("~port", 9559)
        connection_url = f"tcp://{robot_ip}:{port}"
        app = qi.Application(["ReactToTouch", "--qi-url=" + connection_url])
        rospy.loginfo(f"Connected to NAOqi at {robot_ip}:{port}")
    except RuntimeError:
        rospy.logerr("Failed to connect to NAOqi. Check IP and port.")
        sys.exit(1)

    react_to_touch = ReactToTouch(app)
    react_to_touch.run()
