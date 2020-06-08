#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_capture:
    '''
    Image capture and ROS publisher
    '''

    def __init__(self, node_name="image_capture"):
        '''
        Initialize the image capturer

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.node_name = node_name
        rospy.init_node(node_name)
        cam_topic = rospy.get_namespace() + "cv2_capture"
        self.image_pub = rospy.Publisher(cam_topic, Image, queue_size=1)

        self.bridge = CvBridge()

        #self.cv_capture = cv2.VideoCapture('v4l2src device=/dev/video0 ! video/x-raw, framerate=30/1, width=640, height=360 ! videoconvert ! appsink')
        self.cv_capture = cv2.VideoCapture(0)
        print('Initiated cam connection')

    def run(self):
        try:
            while not rospy.is_shutdown():
                ret, cv_img = self.cv_capture.read()
                if ret:  # Publish the resulting frame
                    try:
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "passthrough"))
                    except CvBridgeError as e:
                        print(e)
        except rospy.ROSInterruptException:
            pass
    
if __name__ == "__main__":
	img_cap = image_capture()
	img_cap.run()
	img_cap.cv_capture.release()
	cv2.destroyAllWindows()
