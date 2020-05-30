#!/usr/bin/env python
import serial
import time
import struct
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray, Float32MultiArray  # dont use
import tf

def writePWM(pwmList):
    global ser
    startWrite = 0xDB
    endWrite = 0xBD
    b = bytearray()
    b.append(startWrite)
    ser.write(b)

    for i in range(4): # 4 pwms in custom message
        pwm_bytes = struct.pack('i', pwmList.data[i])
        
        ser.write(pwm_bytes)
   
    b=bytearray()
    b.append(endWrite)
    ser.write(b)


rospy.init_node("ControlNode")
IMU_topic = rospy.get_namespace() + 'imuRaw'
Enc_topic = rospy.get_namespace() + 'encoders'
PWM_topic = rospy.get_namespace() + 'pwm'

IMU_raw_pub = rospy.Publisher(IMU_topic, Imu, queue_size=10)
encoder_pub = rospy.Publisher(Enc_topic, Float32MultiArray, queue_size = 10)
pwm_sub = rospy.Subscriber(PWM_topic, Int32MultiArray, writePWM)
imu_msg = Imu()
encoder_msg = Float32MultiArray() 

ser = serial.Serial("/dev/ttyACM0",9600)


while not rospy.is_shutdown():
    if(ser.in_waiting):
        SYNC = hex(ord(ser.read()))
        if (SYNC == '0xda'):
            motor1_freq = ser.read(4)
            motor1_freq = struct.unpack('<f', motor1_freq)[0]
            motor2_freq = ser.read(4)
            motor2_freq = struct.unpack('<f', motor2_freq)[0]
            accel_x = ser.read(4)
            accel_x = struct.unpack('<f', accel_x)[0]
            accel_y = ser.read(4)
            accel_y = struct.unpack('<f', accel_y)[0]
            accel_z = ser.read(4)
            accel_z = struct.unpack('<f', accel_z)[0]
            mag_x = ser.read(4)
            mag_x = struct.unpack('<f', mag_x)[0]
            mag_y = ser.read(4)
            mag_y = struct.unpack('<f', mag_y)[0]
            mag_z = ser.read(4)
            mag_z = struct.unpack('<f', mag_z)[0]
            gyro_x = ser.read(4)
            gyro_x = struct.unpack('<f', gyro_x)[0]
            gyro_y = ser.read(4)
            gyro_y = struct.unpack('<f', gyro_y)[0]
            gyro_z = ser.read(4)
            gyro_z = struct.unpack('<f', gyro_z)[0]

            STOP = hex(ord(ser.read(1)))
            if (STOP == "0xad"):
                #encoder_msg.header.stamp = rospy.Time.now()
                encoder_msg.data = [motor1_freq,motor2_freq,motor1_freq,motor2_freq]
                #imu_msg.header.stamp = rospy.Time.now()
                
                imu_msg.linear_acceleration.x = accel_x # accel
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z
                
                imu_msg.angular_velocity.x = gyro_x # gyro
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z
                
                #b = bytearray(struct.pack('f',gyro_z))
                #print(["0x%02x" % p for p in b])
                yaw = math.atan2(mag_y, mag_x)

                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]
                
                encoder_pub.publish(encoder_msg)
                IMU_raw_pub.publish(imu_msg)
    
    time.sleep(.001)   

ser.close()     
    #check in_waiting
    #check sync start byte
    #read data (floats 4 bytes)
    #check stop byte for validation

