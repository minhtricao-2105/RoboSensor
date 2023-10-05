import rospy
import cv2
import os
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    global image_counter
    global saved_image_counter
    
    # Only save every Nth image
    N = 100
    
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Only save the image if the counter is a multiple of N
    if image_counter % N == 0:
        img_filename = f"extracted_images/frame_{saved_image_counter}.png"
        cv2.imwrite(img_filename, cv_image)
        saved_image_counter += 1
    
    image_counter += 1

if __name__ == '__main__':
    # Define the topic and bag file
    bag_file = "my_images.bag"
    topic = "/camera/color/image_raw"
    
    # Ensure the output directory exists
    if not os.path.exists('extracted_images'):
        os.makedirs('extracted_images')
    
    # Initialize the node and the bridge
    rospy.init_node('image_extractor', anonymous=True)
    bridge = CvBridge()
    image_counter = 0
    saved_image_counter = 0
    
    # Subscribe to the topic
    rospy.Subscriber(topic, Image, callback)
    
    # Open the bag file in a subprocess and extract images
    try:
        # Playing the ROS bag using subprocess
        process = subprocess.Popen(['rosbag', 'play', bag_file])
        rospy.spin()  # Keep the node running until it's shut down
    except KeyboardInterrupt:
        print("Image extraction interrupted")
    finally:
        # Ensure that the rosbag play process is terminated
        process.terminate()
