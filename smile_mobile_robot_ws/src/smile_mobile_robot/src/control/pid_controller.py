#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/26/2020
Description: Generalized PID controller class.
'''
import sys
import os
import time
import math

class PID_Controller():
    '''
    PID controller
    '''
    def __init__(self, k_p=0.0, k_i=0.0, k_d=0.0, min_error=-1.0e3, max_error=1.0e3,
                 min_control_effort=-1.0e3, max_control_effort=1.0e3,
                 integral_min=-1.0e3, integral_max=1.0e3, angle_error=False,
                 angle_type='rad'):
        '''
        Initialize the pid controller

        Parameters:
            k_p: Proportional gain. Default: 0.0
            k_i: Integral gain. Default: 0.0
            k_d: Derivative gain. Default: 0.0
            min_error: The minimum error between set_point(SP) and process variable
                        (PV). Default: -1.0e3
            max_error: The maximum error between SP and PV. Default: 1.0e3
            min_control_effort: The minimum value of the control effor output:
                        Defualt: -1.0e3
            max_control_effort: The maximum value of the control effort output.
                        Default: 1.0e3
            integral_min: The minimum integral error. Default: -1.0e3
            integral_max: The maximum integral error. Default: 1.0e3
            angle_error: Takes a Boolean. If true, then the error calculation will
                    account for angle wrapping. Default: False.
            angle_type: Takes either value of 'rad' for radians or 'deg' for degrees.
                        If radians, than the expected input range is [-pi, pi].
                        If degrees, than the expected input range is [-180, 180].
        '''
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.min_error = min_error
        self.max_error = max_error
        self.min_control_effort = min_control_effort
        self.max_control_effort = max_control_effort
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.angle_error = angle_error
        self.angle_type = angle_type

        #Previous step in update
        self.prev_time = time.time()

        #correction step contributions from Kp, Ki, Kd
        self.c_p, self.c_i, self.c_d = 0.0, 0.0, 0.0
        self.prev_error = 0.0

    def set_gains(self, k_p, k_i, k_d):
        '''
        Set the gains of the PID controller

        Parameters:
            k_p: Proportional gain.
            k_i: Integral gain.
            k_d: Derivative gain.
        Returns:
            N/A
        '''
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

    def bound_value(self, value, min, max):
        '''
        Helper function for bounding a value between a minimum and maximum
        value.

        Parameters:
            value: The value to bound
            min: The minimum value to take.
            max: The maximum value to take
        Returns:
            result: The new value within the range
        '''
        if(value < min):
            return(min)
        if(value > max):
            return(max)
        return(value)

    def update(self, set_point, process_point):
        '''
        Update step for the pid controller to make a control effort

        Parameters:
            set_point: The desired control point.
            process_point: The current measured point.
        Returns:
            control_effort: The control output for correction.
            error: The bounded error.
        '''

        #Time in-between update steps
        curr_time = time.time()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        error = set_point - process_point

        #Check if angle error correction is need.
        if(self.angle_error):
            #Radians
            if(self.angle_type == 'rad'):
                #Account for angle wrap if necessary
                if(error > math.pi):
                    error = error - (2.0*math.pi)
                elif(error < -1.0 * math.pi):
                    error = error + (2.0*math.pi)
            #Degrees
            else:
                #Account for angle wrap if necessary
                if(error > 180.0):
                    error = error - 360.0
                elif(error < -180.0):
                    error = error + 360.0
        #Bound errors if needed
        error = self.bound_value(error, self.min_error, self.max_error)

        #Calculate the corrections
        self.c_p = error
        self.c_i = self.c_i + (error*dt)
        self.c_d = (error - self.prev_error) / dt

        #Bound the integral if needed
        self.c_i = self.bound_value(self.c_i, self.integral_min, self.integral_max)

        control_effort = (self.k_p * self.c_p) \
                       + (self.k_i * self.c_i) \
                       + (self.k_d * self.c_d)
        #Bound control effort if needed
        control_effort = self.bound_value(control_effort, self.min_control_effort, self.max_control_effort)
        return(control_effort, error)

if __name__ == "__main__":
    pid_controller = PID_Controller(k_p=1.0, k_i=1.0, angle_error=True, angle_type='deg')

    while(True):
        [control_effort, error] = pid_controller.update(180.0, -179.0)
        print('Control Effort:', control_effort, 'Error:', error)
        time.sleep(0.1)
