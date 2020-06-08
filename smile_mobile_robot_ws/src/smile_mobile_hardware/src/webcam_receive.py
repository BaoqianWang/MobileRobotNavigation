import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_capture:
    '''
    Image capture and ROS Subscriber
    '''

    def __init__(self, node_name="image_sub"):
        '''
        Initialize the image Subscriber

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.node_name = node_name
        rospy.init_node(node_name)
        cam_topic = rospy.get_namespace() + "cv2_capture"
        self.image_sub = rospy.Subscriber(cam_topic, Image, self.display)

        self.bridge = CvBridge()

    
    def display(self, ros_img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_img, "passthrough")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Image window", cv_image)


    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        img_cap.cv_capture.release()
        cv2.destroyAllWindows()
if __name__ == '__main__':
	img_cap = image_capture()
	img_cap.run()
	cv2.destroyAllWindows()
