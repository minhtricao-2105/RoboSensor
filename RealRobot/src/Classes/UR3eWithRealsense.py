import rosbag, rospy
from .UR3e import UR3e
from sensor_msgs.msg import Image

class UR3eWithRealSense(UR3e):
    
    def __init__(self, filename):
        super().__init__()
        
        # Initialize rosbag attributes
        self.bag = rosbag.Bag(filename, 'w')
        # self.realsense_topic = "/camera/color/image_raw"
        self.realsense_topic = "/camera/color/image_raw"

        
        # The subscriber will be initialized when needed and closed after each frame capture
        self.subscriber = None  

    def capture_single_frame(self):
        """Capture a single frame from the Realsense camera and write to the rosbag."""
        self.subscriber = rospy.Subscriber(self.realsense_topic, Image, self._record_frame_and_unregister)

        print("Capturing Image...")

    def _record_frame_and_unregister(self, msg):
        """
        Private callback for recording a single frame into the rosbag and then unregistering the subscriber.
        """
        self.bag.write(self.realsense_topic, msg)
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None

    def close_bag(self):
        """
        Close the rosbag. This should be done when you're completely done recording.
        """
        if self.bag:
            self.bag.close()
