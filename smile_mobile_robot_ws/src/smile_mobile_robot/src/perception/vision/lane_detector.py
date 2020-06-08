#!/usr/bin/env python
'''
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Date Created:06/01/2020
Description: OpenCV based lane detector. Algorithm adpated from https://www.hackster.io/kemfic/curved-lane-detection-34f771
'''
import numpy as np
import cv2
import glob
import time
import pickle
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

class Lane_Detector():
    '''
    OpenCV based lane detector including calibration.
    '''
    def __init__(self):
        '''
        Initialize the lane detector.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        self.sobel_x_thresh = [70, 255]
        self.sat_thresh = [10, 255]
        self.num_windows = 9
        self.margin = 150
        self.min_lane_pix = 2 #Minimum number of pixels detected as a lane in the sliding windows.
        self.center_line = 400 #The line to plot the trajectory point.(with respect to topdown view)
        self.center_line_region = [50, 50] #The number of vertical pixels above and below the center line.
        self.min_lane_candidate_pix = 14000 #The number of possible pixels on the left and right side that helps validate there is a likely lane.


    def calibrate_camera(self, grid_count_h, grid_count_v, cal_save_file):
        '''
        Calibrate the camera to minimize distortion.

        Parameters:
            grid_count_h: The number of horizontal grids
            grid_count_v: The number of vertical grids.
            cal_save_file: The file to save the camera calibrations. should have an ending
                        of ".p"
        Returns:
            N/A
        '''

        #prepate the object points on that will be placed on the calibration grid
        #makes the grid with (x, y, z) points where z=0
        object_points = np.zeros((grid_count_h*grid_count_v, 3), np.float32)
        object_points[:, :2] = np.mgrid[0:grid_count_h, 0:grid_count_v].T.reshape(-1, 2)

        #Array to stor the object points matrix
        object_points_array = []
        image_points_array = [] #2D corner points found by the chess board find function


        #For testing purposes, grab calibration images.
        images = glob.glob('calibration_imgs/*.jpg')

        for frame_name in images:

            #For testing purposes.
            img = cv2.imread(frame_name)
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            cv2.imshow("image", img)
            cv2.waitKey(10)
            print(img.shape)
            #Find the chess board
            ret, corners = cv2.findChessboardCorners(gray_img, (grid_count_h, grid_count_v), None)

            #If the chess board id found, save the points identified
            if(ret == True):
                print("Here")
                object_points_array.append(object_points)
                image_points_array.append(corners)
            time.sleep(0.5)
        image_size = (img.shape[1], img.shape[0])

        #Calibrate the camera. Provides camera matrix, distortion coefficients, rot + trans. vectors.
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points_array, image_points_array, image_size, None, None)

        #Pickle the camera calibrations and save to a file.
        dist_pickle = {}
        dist_pickle['mtx'] = mtx
        dist_pickle['dist'] = dist
        pickle.dump(dist_pickle, open(cal_save_file, 'wb'))

        #Save the camera matrix and distortion matrix
        self.camera_matrix = mtx
        self.distortion_matrix = dist


    def load_calibrations(self, cal_save_file):
        '''
        Load the camera calibrations that are used to undistor images.

        Parameters:
            cal_save_file: The file were the pickled camra matrix and distortion matrix
                        are saved for camera calibrations. Give entire path.

        Returns:
            N/A
        '''

        with open(cal_save_file, mode="rb") as f:
            file = pickle.load(f)
        self.camera_matrix = file['mtx']
        self.distortion_matrix = file['dist']

    def undistort_image(self, img):
        '''
        Undistort the image. Performs a hard copy of the new undistored image.

        Parameters:
            img: The numpy img to undistor
        Returns:
            dst: The undistored image.
        '''
        dst = cv2.undistort(img, self.camera_matrix, self.distortion_matrix, None, self.camera_matrix)

        return(dst)


    def _perspective_warp(self, img):

        '''
        Change the perspective of the image to mimic a top down view of the road.

        Parameters:
            img: The image to change the perspective on. Warp the image.
        Returns:
            warped: The warped image.
        '''

        #Apply the transformation matrix and warp the image
        warped = cv2.warpPerspective(img, self.M, tuple(self.dst_size))
        return warped
    def _inv_perspective_warp(self, img):

        '''
        Change the perspective of the image to mimic a top down view of the road.

        Parameters:
            img: The image to change the perspective on. Warp the image.
        Returns:
            warped: The warped image.
        '''

        #Apply the transformation matrix and warp the image
        warped = cv2.warpPerspective(img, np.linalg.inv(self.M), tuple(self.dst_size))
        return warped


    def _initialize_perspective_warp(self, src_size, dst_size, src_roi):
        '''
        Initilize the region of interest for the lane detector to look for lanes.

        Parameters:
            src_size: The size of the source image.
            dst_size: The size of the desired warped image.
            src_roi: The region of interest from the source lane image to look for the lane.
                    These should be given in a percentage format.
                    Order [Top left point, top right point, bottom left point, bottom right point]
                        Ex:
                        src_roi = np.float32([(0.43, 0.65), (0.58. 0.65), (0.1, 1), (1, 1)])
        Returns:
            N/A
        '''
        #Keep the destination image the same size as the source image.
        dst_roi = np.float32([(0, 0), (1, 0), (0, 1), (1, 1)])
        dst = np.float32(src_size) * dst_roi
        src = np.float32(src_size) * src_roi
        self.src_roi = src_roi

        #Get the transformation matrix to switch perspective on the image.
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.src_size = src_size
        self.dst_size = dst_size

        return

    def show_roi(self, img):
        '''
        Draw the region of interest on a lane image.

        Parameters:
            img: The image to draw the region on.

        Returns:
            N/A
        '''
        roi_img = np.copy(img)
        src = np.float32(self.src_size) * self.src_roi

        #Rearrange the ordering of points for the polyline draw. Draw in a clockwise fashion.
        src = np.float32([src[2], src[0], src[1], src[3]])
        src = np.int32(src).reshape((-1, 1, 2))

        cv2.polylines(roi_img, [src], True, (0, 255, 0))

        return roi_img

    def binarize_img(self, img, undistort=False):
        '''
        Binarizes the RGB lane image using sobel filtering on the HLS
        color format.

        Parameters:
            img:

        Returns:
        '''
        if(undistort):
            img = self.undistort_image(img)

        #Convert the image to HLS space and seperate out V channel
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS).astype(np.float)
        l_channel = hls[:, :, 1] #Lightness
        s_channel = hls[:, :, 2] #Saturation
        h_channel = hls[:, :, 0] #Hue

        #Take the sobel in the vertical (x) direction in the light channel
        #Output is float64.
        sobel_x = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0)
        abs_sobel_x = np.absolute(sobel_x)

        #Scale to 8bit range for the next binarization part
        scaled_sobel_x = np.uint8(255 * abs_sobel_x/np.max(abs_sobel_x))

        #Threshold x gradient
        sobel_x_binary = np.zeros_like(scaled_sobel_x)
        sobel_x_binary[(scaled_sobel_x >= self.sobel_x_thresh[0]) & (scaled_sobel_x <= self.sobel_x_thresh[1])] = 1

        #Help elimate white noise
        kernel_1 = np.ones((3, 3), np.uint8)
        sobel_x_binary = cv2.morphologyEx(sobel_x_binary, cv2.MORPH_OPEN, kernel_1)
        kernel_2 = np.ones((5, 5), np.uint8)
        sobel_x_binary = cv2.dilate(sobel_x_binary, kernel_2, iterations=1)

        #Threshold color channel (Take the brighter colors like white and yellow)
        sat_binary = np.zeros_like(s_channel)
        sat_binary[(s_channel >= self.sat_thresh[0]) & (s_channel <= self.sat_thresh[1])] = 1

        #Concatenate the binarized images in the depth direction to make a 3 channel colored image.
        #NOTE: Note sure if this is used, may be more for visual purposes
        color_binary = np.dstack((np.zeros_like(sobel_x_binary), sobel_x_binary, sat_binary)) * 255

        #Make a combined binary image that is the Union
        combined_binary = np.zeros_like(sobel_x_binary)
        combined_binary[(sobel_x_binary == 1) | (sat_binary == 1)] = 1

        return(sobel_x_binary)

    def histogram(self, img):
        '''
        Get a historgram of the columns of pixels for helping detect the lane

        Parameters:
            img: A binary image that should have the lanes as 1's and rest as 0's
        Returns
            hist: The histogram bins
        '''
        hist = np.sum(img[img.shape[0]//2:, :], axis=0)
        return(hist)

    def sliding_window(self, img, draw_windows=True):
        '''
        Slide windows over the primary regions where the lanes are found. A histogram
        determines the optimal region to slide windows.

        Parameters:
            img: The perspective warped, "top-down" view image of the lanes
            draw_windows: Draw the windows on the output image.
        Returns:
            out_img: The ouput image with the drawn windows
        '''

        #Set up the output image
        out_img =np.dstack((img, img, img)) * 255

        #Get the histogram of the image.
        histogram = self.histogram(img)

        #find the columns in the img that have the peaks in the histogram
        #Most likely candidate positions for lane
        midpoint = int(histogram.shape[0]/2)
        num_left_candidate_pixels = np.sum(histogram[:midpoint])
        num_right_candidate_pixels = np.sum(histogram[midpoint:])
        if(num_left_candidate_pixels < self.min_lane_candidate_pix or num_right_candidate_pixels < self.min_lane_candidate_pix):
            return(False, None, None, None, None)

        left_peak_column = np.argmax(histogram[:midpoint])
        right_peak_column = np.argmax(histogram[midpoint:]) + midpoint


        #Set the window height
        window_height = np.int(img.shape[0]/self.num_windows)

        #Identify the x and y positions of all nonzero pixels in the image
        nonzero = img.nonzero()
        nonzero_x = np.array(nonzero[1])
        nonzero_y = np.array(nonzero[0])

        #Set the positions to start iterating over in the x direction
        left_x_curr = left_peak_column
        right_x_curr = right_peak_column

        #Create empty list to receive left and right lane pixel indices
        left_lane_indexs = []
        right_lane_indexs = []

        for window in range(self.num_windows):

            #Identify window boundaries in x and y (and right and left)
            win_y_low = img.shape[0] - (window+1) * window_height
            win_y_high = img.shape[0] - window * window_height

            win_x_left_low = left_x_curr - self.margin
            win_x_left_high = left_x_curr + self.margin
            win_x_right_low = right_x_curr - self.margin
            win_x_right_high = right_x_curr + self.margin

            #Draw the sliding windows for the left and right lanes
            if draw_windows == True:

                cv2.rectangle(out_img,(win_x_left_low,win_y_low),(win_x_left_high,win_y_high),
                             (100,255,255), 3)
                cv2.rectangle(out_img,(win_x_right_low,win_y_low),(win_x_right_high,win_y_high),
                             (100,255,255), 3)

            #Identify the nonzero pixels in x and y within the window
            good_left_indexs = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & \
                (nonzero_x >= win_x_left_low) &  (nonzero_x < win_x_left_high)).nonzero()[0]
            good_right_indexs = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & \
                (nonzero_x >= win_x_right_low) &  (nonzero_x < win_x_right_high)).nonzero()[0]

            #Append these indices to the lists
            left_lane_indexs.append(good_left_indexs)
            right_lane_indexs.append(good_right_indexs)

            #Shift the next sliding window location by finding the mean of the nonzero pixels
            #found within the window.
            if len(good_left_indexs) > self.min_lane_pix:
                left_x_curr = np.int(np.mean(nonzero_x[good_left_indexs]))
            if len(good_right_indexs) > self.min_lane_pix:
                right_x_curr = np.int(np.mean(nonzero_x[good_right_indexs]))

        #Concatenate the array indicies
        left_lane_indexs = np.concatenate(left_lane_indexs)
        right_lane_indexs = np.concatenate(right_lane_indexs)


        #Extract the left and righ line pixel positions
        left_x = nonzero_x[left_lane_indexs]
        left_y = nonzero_y[left_lane_indexs]
        right_x = nonzero_x[right_lane_indexs]
        right_y = nonzero_y[right_lane_indexs]

        if(left_x.size == 0 or left_y.size == 0 or right_x.size == 0 or right_y.size == 0):
            return False, None, None, None, None
        #Fit a second order ploynomial
        left_fit = np.polyfit(left_y, left_x, 2)
        right_fit = np.polyfit(right_y, right_x, 2)

        #TODO: See how to solve issue with bad lanes.
        #Possibly use the covariance?

        left_a = [left_fit[0]]
        left_b = [left_fit[1]]
        left_c = [left_fit[2]]

        right_a = [right_fit[0]]
        right_b = [right_fit[1]]
        right_c = [right_fit[2]]

        left_fit_ = [0, 0, 0]
        right_fit_ = [0, 0, 0]

        left_fit_[0] = np.mean(left_a[-10:])
        left_fit_[1] = np.mean(left_b[-10:])
        left_fit_[2] = np.mean(left_c[-10:])

        right_fit_[0] = np.mean(right_a[-10:])
        right_fit_[1] = np.mean(right_b[-10:])
        right_fit_[2] = np.mean(right_c[-10:])

        # Generate x and y values for plotting from the poly (ax^2 +bx +c)
        plot_y = np.linspace(0, img.shape[0]-1, img.shape[0] )
        left_fit_x = left_fit_[0] * plot_y**2 + left_fit_[1] * plot_y + left_fit_[2]
        right_fit_x = right_fit_[0]*plot_y**2 + right_fit_[1]*plot_y + right_fit_[2]

        out_img[nonzero_y[left_lane_indexs], nonzero_x[left_lane_indexs]] = [255, 0, 100]
        out_img[nonzero_y[right_lane_indexs], nonzero_x[right_lane_indexs]] = [0, 100, 255]

        return True, out_img, (left_fit_x, right_fit_x), (left_fit_, right_fit_), plot_y

    def draw_lanes(self, img, left_fit, right_fit):
        '''
        Draw the lanes detected by the lane detector.

        Parameters:
            img: The warped "top-down" viewed image of the lanes
            left_fit: The polynomial points describing the left lane
            right_fit: The polynomial points describing the right lane.
        Returns:
            out_imag: The original perspective of the lane with the detected lanes outlined
        '''
        plot_y = np.linspace(0, img.shape[0] - 1, img.shape[0])
        color_img = np.zeros_like(img)

        left = np.array([np.transpose(np.vstack([left_fit, plot_y]))])
        right = np.array([np.flipud(np.transpose(np.vstack([right_fit, plot_y])))])
        points = np.hstack((left, right))

        #Paint the detected lane
        cv2.fillPoly(color_img, np.int_(points), (51, 255, 51))

        #Get the center line trajectory
        center_fit, center_point = self.get_trajectory_point(left_fit, right_fit)
        center = np.array([np.transpose(np.vstack([center_fit, plot_y]))])
        cv2.polylines(color_img, np.int_(center), isClosed=False, color=(255, 0, 0), thickness=3)

        #cv2.circle(color_img, (center_point, self.center_line), 10, (0, 162, 255), thickness=-1)
        cv2.line(color_img, (center_point-20, self.center_line), (center_point+20, self.center_line), (255, 0, 0), thickness=10)

        color_img = self.draw_vehicle_trajectory_line(color_img, self.center_line)

        out_img = self._inv_perspective_warp(color_img)
        out_img = cv2.addWeighted(img, 1, out_img, 0.7, 0)

        return(out_img)

    def draw_vehicle_trajectory_line(self, img, center_line_region):
        '''
        Draw the trajectory line on the lane detection image. This is like the center of the
        camera.

        Parameters:
            img: The lane image
            center_line_region: The horizontal line considered the center.
        Returns:
            out_img: The lane image with the drawn trajectory line.
        '''
        out_img = np.copy(img)

        #Draw the trajector line
        cv2.arrowedLine(out_img, (img.shape[1]//2, img.shape[0]), (img.shape[1]//2, center_line_region), (0, 0, 255), thickness=4)

        return(out_img)


    def get_trajectory_point(self, left_fit_x, right_fit_x):
        '''
        Get the center of the lane and the trajectory (point) needed
        to follow the lane.

        Parameters:
            left_fit_x: The polynomial fitting the left lane.
            right_fit_x: The polynomial fitting the right lane.

        Returns:

        '''

        #Take the mean of the polynomials
        center_fit = (left_fit_x + right_fit_x) / 2.0

        #Calculate the center within the center line region
        left_fit_in_center = left_fit_x[self.center_line - self.center_line_region[0]:self.center_line + self.center_line_region[1]]

        right_fit_in_center = right_fit_x[self.center_line - self.center_line_region[0]:self.center_line + self.center_line_region[1]]

        center_point = np.mean((left_fit_in_center + right_fit_in_center) / 2.0)


        return center_fit, int(center_point)






    def detect(self, img):
        '''
        The entire pipeline for detecting the lane.

        Parameters:
            img: The raw input BGR image of the lane/road.

        Returns:
            final_img: The image with the lane traced over it.
        '''

        #Change the perspective to have a pseudo "top-down" view of the lanes
        warped_img = self._perspective_warp(img)

         #Binarized the image using sobel filters to identify most probale regions for the lane
        binary_img = self.binarize_img(warped_img)

        #return(warped_img)
        #Perform sliding window to detect the lane curvatures
        ret, slide_img, curves, lanes, plot_y = self.sliding_window(binary_img, draw_windows=True)

        #If a "good" lane detection was found, draw the lanes
        if(ret == True):
            #Draw the lane region on a new colored picture.
            final_img = self.draw_lanes(img, curves[0], curves[1])
            _, center_point = self.get_trajectory_point(curves[0], curves[1])
            return ret, final_img, center_point

        return False, img, None

class Lane_Detection_ROS():
    """
    This class provides the interface between the Lane detection algorithm
    and the smile mobile robot running in the ROS ecosystem.
    """
    def __init__(self, node_name="lane_detector"):
        '''
        Initialize the communication to the image data and run lane detection on it.

        Parameters:
            node_name: The name of this ROS lane detection node. Default: lane_detector

        Returns:
            N/A
        '''

        rospy.init_node(node_name)

        #TOPICS
        camera_topic = rospy.get_namespace() + 'camera1/image_raw'
        lane_tracking_topic = rospy.get_namespace() + 'lane_tracking'

        #Subscriber to the image data coming from the robot
        self.camera_sub = rospy.Subscriber(camera_topic, Image, self._camera_callback)

        #Convesion bridge between ROS image messages and OpenCV message types.
        self.bridge = CvBridge()

        self._initialize_lane_detector()

        #Initialize the publisher for sending the the difference between the lane center
        #and robot center.
        self.lane_tracking_pub = rospy.Publisher(lane_tracking_topic, Float32MultiArray, queue_size=10)

        self.rate = rospy.Rate(30) #30Hz fps

    def _initialize_lane_detector(self):
        '''
        Initialize all the components of the lane detector by loading in
        the parameters from the parameter server.

        Parameters:
            N/A
        Returns:
            N/A
        '''

        #Initialize the lane detector algorithm
        self.lane_detector = Lane_Detector()

        param_path = '/vision_params/lane_detector/'

        self.img_size = rospy.get_param(param_path + 'img_size')

        #Generate a blank image
        self.img = np.ones((self.img_size[0], self.img_size[1], 3), np.uint8)

        self.lane_detector.sobel_x_thresh = rospy.get_param(param_path + 'sobel_x_thresh')
        self.lane_detector.sat_thresh = rospy.get_param(param_path + 'sat_thresh')
        self.lane_detector.num_windows = rospy.get_param(param_path + 'num_windows')
        self.lane_detector.margin = rospy.get_param(param_path + 'margin')
        self.center_line = rospy.get_param(param_path + 'center_line')
        self.lane_detector.center_line = self.center_line
        self.lane_detector.center_line_region = rospy.get_param(param_path + 'center_line_region')
        src_roi = rospy.get_param(param_path + 'src_roi')
        self.src_roi = np.float32(src_roi)
        self.lane_detector.src_roi = self.src_roi

        self.lane_detector._initialize_perspective_warp(self.img_size, self.img_size, self.src_roi)

    def _camera_callback(self, img_msg):
        '''
        Callback for the image data being received

        Parameters:
            img_msg: The image data message
        Returns:
            N/A
        '''
        try:
            #Converts the image message to a cv2 (numpy) message
            self.img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def run(self):
        '''
        Main loop for running the ros interface for lane detection

        Parameters:
            N/A
        Returns:
            N/A
        '''

        try:
            while not rospy.is_shutdown():

                ret, lane_detect_img, center_point = self.lane_detector.detect(self.img)
                #roi = self.lane_detector.show_roi(self.img)
                #cv2.imshow("ROI", roi)
                #warped = self.lane_detector._perspective_warp(self.img)
                #binary = self.lane_detector.binarize_img(warped)
                #cv2.imshow("Binary", binary*255)

                show_img = self.img

                #The data need to perform lane following.
                #[valid_lane_bool, center_of_camera_trajector_line, center_point_of_lane_detected]
                lane_tracking_msg = Float32MultiArray()
                lane_tracking_msg.data = [0, 0.0, 0]


                #If a valid lane is detected
                if(ret):
                    show_img = lane_detect_img
                    lane_tracking_msg.data = [1, self.img_size[1]//2, center_point]

                #Publish the lane tracking information to follow the lane
                self.lane_tracking_pub.publish(lane_tracking_msg)

                cv2.imshow("Lane Detection", self.lane_detector.show_roi(show_img))


                cv2.waitKey(1)
                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":

    lane_detection_ros = Lane_Detection_ROS()
    lane_detection_ros.run()
