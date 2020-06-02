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
    
        pass
    
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
        print(src)

        #Get the transformation matrix to switch perspective on the image.
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.src_size = src_size
        self.dst_size = dst_size

        return

    def _show_roi(self, img):
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

    def detect(self, img, undistort=False):
        '''
        Pipeline to do lane detection. 

        Parameters:
            
        Returns:
        '''
        if(undistort):
            img = self.undistort_image(img)
        
        #Convert the image to HLS space and seperate out V channel
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]
        h_channel = hls[:, :, 0]

        #Take the sobel in the vertical (x) direction in the light channel
        sobel_x = cv2.Sobel(l_channel, cv2.CV_64F, 1, 1) 
        abs_sobel_x = np.absolute(sobel_x)

        scaled_sobel = np.uint8(255 * abs_sobel_x/np.max(abs_sobel_x))

        #Trheshold x gradient
        sx_binary = np.zeros_like(scaled_sobel)
        sx_binary = [(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1

        #Threshold color channel
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh[0] & (s_channel <= s_thresh[1])] = 1


if __name__ == "__main__":
    
    lane_detector = Lane_Detector()
    
    src_roi = np.float32([(0.43, 0.62), (0.55, 0.62), (0.1, 1), (1, 1)])
    
     #Get a test image
    img = cv2.imread('test_images/lane_1.jpg')
    img_size = [img.shape[1], img.shape[0]]
   

    lane_detector._initialize_perspective_warp(img_size, img_size, src_roi)
    
    #Show the region that will be extracted to look
    roi_img = lane_detector._show_roi(img)
    cv2.imshow('Lane Image', roi_img)
    
    #Warp the lane image to mimic a top down view
    warped_img = lane_detector._perspective_warp(img)

    cv2.imshow("Warped Lane Image", warped_img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()




    
