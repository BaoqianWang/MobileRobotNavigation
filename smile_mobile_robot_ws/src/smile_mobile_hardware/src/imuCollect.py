import serial
import time
import struct
import rospy
from sensor_msgs.msg import Imu
#from std_msgs.msg import Float32MultiArray  # dont use
#import smile_float_array.msg #float32 array of 4
#import smile_pwm.msg

rospy.init_node("ControlNode")
pubIMURaw = rospy.Publisher('imuRaw', Imu, queue_size=10)
#pubEncoder = rospy.Publisher('encoderArray', smile_float_array, queue_size = 10)
pwmSub = rospy.Subscriber('pwm', smile_pwm, writePWM, )
imuMsg = Imu()
#encoderMsg = smile_float_array() 

ser = serial.Serial("/dev/ttyUSB0",9600)

def writePWM(pwmList)
    startWrite = 0xDB
    endWrite = 0xBD
    ser.write(startWrite)
    for pwm in range(4): # 4 pwms in custom message
        ser.write(pwmList[pwm])
    ser.write(endWrite)


while(1):
    if(ser.in_waiting):
        SYNC = hex(ord(ser.read()))
        if (SYNC == '0xda'):
            motor1_freq = ser.read(4)
            motor1_freq = struct.unpack('<f', motor1freq)
            motor2_freq = ser.read(4)
            motor2_freq = struct.unpack('<f', motor2freq)
            motor3_freq = ser.read(4)
            motor3_freq = struct.unpack('<f', motor3freq)
            motor4_freq = ser.read(4)
            motor4_freq = struct.unpack('<f', motor4freq)
            accel_x = ser.read(4)
            accel_x = struct.unpack('<f', accel_x)
            accel_y = ser.read(4)
            accel_y = struct.unpack('<f', accel_y)
            accel_z = ser.read(4)
            accel_z = struct.unpack('<f', accel_z)
            mag_x = ser.read(4)
            mag_x = struct.unpack('<f', mag_x)
            mag_y = ser.read(4)
            mag_y = struct.unpack('<f', mag_y)
            mag_z = ser.read(4)
            mag_z = struct.unpack('<f', mag_z)
            gyro_x = ser.read(4)
            gyro_x = struct.unpack('<f', gyro_x)
            gyro_y = ser.read(4)
            gyro_y = struct.unpack('<f', gyro_y)
            gyro_z = ser.read(4)
            gyro_z = struct.unpack('<f', gyro_z)

            STOP = hex(ord(ser.read(1)))
            if (STOP == "0xad"):
                #print([accel_x,accel_y,accel_z])
                #print([motor1_freq,motor2_freq,motor3_freq,motor4_freq])
                print(motor1_freq)
                #encoderMsg = [motor1_freq,motor2_freq,motor3_freq,motor4_freq]
                imuMsg.header.stamp= rospy.Time.now()
                imuMsg.header.frame_id = 'base_link'                
                imuMsg.linear_acceleration.x = accel_x # tripe axis accelerator meter
                imuMsg.linear_acceleration.y = accel_y
                imuMsg.linear_acceleration.z = accel_z
                imuMsg.angular_velocity.x = mag_x
                imuMsg.angular_velocity.y = mag_y
                imuMsg.angular_velocity.z = mag_z
                imuMsg.orientation.x = gyro_x #magnetometer
                imuMsg.orientation.y = gyro_y
                imuMsg.orientation.z = gyro_z
                pubIMURaw.publish(imuMsg)
                print(imuMsg)
                #print([mag_x,mag_y,mag_z])
                #print([gyro_x,gyro_y,gyro_z])
                print()
    rospy.spin()
    time.sleep(.001)   

ser.close()     
    #check in_waiting
    #check sync start byte
    #read data (floats 4 bytes)
    #check stop byte for validation

