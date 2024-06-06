from .correctSensor_v6 import correctSensor_v6
import time
import threading
import math
from datetime import datetime
import csv
import os

from .icm20948_lib import ICM20948
from .icm20948_lib import I2C_ADDR_ALT
from .icm20948_lib import AK09916_HXL
from .icm20948_lib import AK09916_CNTL2
from .icm20948_lib import ICM20948_ACCEL_XOUT_H

from .AD7994 import AD7994 
from .pni_rm3100 import PniRm3100

from .sun_vect import compute_vect
from .QUEST import quest
from .models import get_sun_and_magnetic_field_in_ecef
from .EKFQ import QuaternionKalmanFilter
from .rtc import gps_utc2utc

from scipy.spatial.transform import Rotation as R
import numpy as np
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros

from std_msgs.msg import Float64MultiArray

class ADS_Suite(Node):
    def __init__(self):
        super().__init__('ads_suite')

        self.publisher_ = self.create_publisher(Quaternion, 'dynamic_quaternion', 10)
        # self.publisherm = self.create_publisher(Float64MultiArray, 'mag_vect', 10)
        self.timer = self.create_timer(1.0 / 37.0, self.kalman_estimation_loop)
        
        #I2C addresses 
        self.mag_addr = 0x21
        self.tri1_addr = 0x20
        self.tri2_addr = 0x24
        self.imu_addr = 0x69

        self.ADS_Header = ['Time', 'MagX', 'MagY', 'MagZ', 'GyroX', 'GyroY', 'GyroZ', 'AccelX', 'AccelY', 'AccelZ', 'Tri11', 'Tri12', 'Tri13',
                    'Tri21', 'Tri22', 'Tri23', 'Lat', 'Lon', 'Alt', 'SunVect', 'MagVect', 'MagRef', 'SunRef', 
                    'Roll', 'Pitch', 'Yaw', 'static_q', 'dynamic_q']

        #Kalman Filter Member Class
        self.ekf = QuaternionKalmanFilter()

        #Initialize Sensor Data Members as 0
        self.gyroX = 0
        self.gyroY = 0
        self.gyroZ = 0

        self.AccelX = 0
        self.AccelY = 0
        self.AccelZ = 0

        self.magX = 0
        self.magY = 0
        self.magZ = 0


        #Naming Pattern tri{triclops # (1 or 2)}{photodiode # {1,2,3}}
        self.tri11 = 0
        self.tri12 = 0
        self.tri13 = 0

        self.tri21 = 0
        self.tri22 = 0
        self.tri23 = 0

        #GPS Values Intialized to FXB Coords
        self.latitude = 42.3
        self.longitude = -83.7
        self.altitude = 0.3
        self.gps_time = 0

        #Time for sampling data timestamp - time string for filename
        self.time_string = 0
        self.time = 0

        #Initialized Observed Vectors - They start entirely Z - facing
        self.sun_vect = np.array([0,0,1])
        self.mag_vect = np.array([0,0,1])

        #Reverence Vectors Initialized as 0
        self.mag_ref = 0
        self.sun_refv = 0

        #Counts the number of items in the active csv file
        self.csv_entries = 0

        #Filenames/locations for active log & csv
        self.filecsv = None
        self.filelog = None

        #Quaternion Estimates Static and Dynamics Initialized to be Entirely Z-facing
        self.static_q = np.array([0, 0, 0, 1])
        self.dynamic_q = np.array([0, 0, 0, 1])

        #Initialize gyro as +- 8g and +- 2000 dps
        self.imu_gyro = ICM20948(i2c_addr=I2C_ADDR_ALT, i2c_bus=1)
        self.imu_gyro.set_accelerometer_full_scale(8)
        self.imu_gyro.set_gyro_full_scale(2000)

        #Initialize rm3100 mag, use sampling rate of 37hz by default
        self.magnetometer = PniRm3100()
        self.magnetometer.assign_device_addr(self.mag_addr)

        self.magnetometer.print_status_statements = False
        self.magnetometer.print_debug_statements = False

        self.magnetometer.assign_xyz_ccr(x_ccr_in=self.magnetometer.CcrRegister.CCR_DEFAULT, 
                                        y_ccr_in=self.magnetometer.CcrRegister.CCR_DEFAULT, 
                                        z_ccr_in=self.magnetometer.CcrRegister.CCR_DEFAULT)

        self.magnetometer.assign_tmrc(self.magnetometer.TmrcRegister.TMRC_37HZ)
        self.magnetometer.write_ccr()
        self.magnetometer.write_config()

        #Initialize ADCs for the two Triclops Sensors
        self.triclops = AD7994(self.tri1_addr)
        self.triclops2 = AD7994(self.tri2_addr)

        #Conversion #'s for Triclops Values
        self.sun_ref = 3
        self.MaxCounts = 1023

        #Initialize First Time
        self.time = time.time()

        #Setup Initial files and directories
        timestamp = datetime.utcnow()
        filecsv = str(self.time) 
        filelog = str(self.time)

        formatted_time = timestamp.strftime('%Y%m%d_%H%M%S')
        folder_name = 'ADS_data'
        folder_name2 = 'ADS_logs'

        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        if not os.path.exists(folder_name2):
            os.makedirs(folder_name2)


        self.filecsv = formatted_time + '_ADS.csv'
        self.filelog = formatted_time + '_ADSlog.txt'

        self.filecsv = os.path.join(folder_name, self.filecsv)
        self.filelog = os.path.join(folder_name2, self.filelog)

        with open(self.filecsv, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.ADS_Header)

        line = "Starting ADS Log\n"
        with open(self.filelog, 'w') as file:
            file.write(line)

    def parse_datetime(datetime_str):
        # Split the date and time parts
        date_part, time_part = datetime_str.split(' - ')
        
        # Split the date into day, month, and year
        day, month, year = map(int, date_part.split('/'))
        
        # Split the time into hour, minute, seconds.milliseconds
        time_details = time_part.split(':')
        hour = int(time_details[0])
        minute = int(time_details[1])
        
        # Split seconds and milliseconds and convert seconds to integer
        second = int(time_details[2].split('.')[0])
        
        # Combine all parts into an array
        return [day, month, year, hour, minute, second]

    def getTriclopsReading(self):
        data = self.triclops.get_data() 
        
        byte_convert = [point * self.sun_ref/self.MaxCounts for point in data]
        self.tri11 = byte_convert[0]
        self.tri12 = byte_convert[1]
        self.tri13 = byte_convert[2]
        

        data2 = self.triclops2.get_data()

        # This exists for spitting out the bytes for the radio
        # byte_string = b''.join([i.to_bytes(2, byteorder='big', signed=False) for i in data])
        # byte_string2 = b''.join([i.to_bytes(2, byteorder='big', signed=False) for i in data2])

        # first_three_bytes = byte_string[:6]
        # first_three_bytes2 = byte_string2[:6]
        # bs = first_three_bytes + first_three_bytes2

        
        byte_convert = [point * self.sun_ref/self.MaxCounts for point in data2]
    
        self.tri21 = byte_convert[0]
        self.tri22 = byte_convert[1]
        self.tri23 = byte_convert[2]
        
        return byte_convert

    def getMagReading(self):
        readings = self.magnetometer.read_meas() 

        # Relative magnetometer readings
        magX = readings[0]
        magY = readings[1]
        magZ = readings[2]

        # MC10 Orientation mag -- Ignore
        self.magX = magX
        self.magY = -magY
        self.magZ = -magZ
        # print(f"mgz: {magZ}")

        # UNCOMMENT FOR CALIBRATED VERSION
        # mags = correctSensor_v6(self.mag_params, magX, magY, magZ)
        # magX = mags[0]
        # magY = mags[1]
        # magZ = mags[2]

        # magXb = readings[3]
        # magYb = readings[4]
        # magZb = readings[5]

        self.mag_vect = np.array([magY, magX, -magZ])
        self.mag_vect = self.mag_vect / np.linalg.norm(self.mag_vect)
        # msg = Float64MultiArray()
        # msg.data = self.mag_vect
        # self.publisherm.publish(msg)

        # magXb = int(self.magX*100)
        # magXb = magXb.to_bytes(4, byteorder='big', signed=True)
        # magYb = int(self.magY*100)
        # magYb = magYb.to_bytes(4, byteorder='big', signed=True)
        # magZb = int(self.magZ*100)
        # magZb = magZb.to_bytes(4, byteorder='big', signed=True)
        # bts = magXb + magYb + magZb

        return readings

    def getGyroReading(self):
        #data = self.imu.read_bytes(ICM20948_ACCEL_XOUT_H + 6, 12) #Return the raw gyro readings, each is a raw 2 bytes
        ax, ay, az, gx, gy, gz = self.imu_gyro.read_accelerometer_gyro_data()
        
        # UNCOMMMENT FOR CALIBRATED VERSION
        # gx -= self.gyro_dc[0]
        # gy -= self.gyro_dc[1]
        # gz -= self.gyro_dc[2]

        # MC10 relative gyro
        # self.gyroX = gy
        # self.gyroY = gx
        # self.gyroZ = -1*gz
        self.gyroX = gx
        self.gyroY = gy
        self.gyroZ = gz
        self.AccelX = ax
        self.AccelY = -ay
        self.AccelZ = -az

        # Gyro to bytes
        # gyroXe = int(gx*100)
        # gyroXe = gyroXe.to_bytes(4, byteorder='big', signed=True)
        # gyroYe = int(gy*100)
        # gyroYe = gyroYe.to_bytes(4, byteorder='big', signed=True)
        # gyroZe = int(gz*100)
        # gyroZe = gyroZe.to_bytes(4, byteorder='big', signed=True)
        # bts = gyroXe + gyroYe + gyroZ

        return gx, gy, gz

    #Currently this just utilizes system time
    def get_time_str(self):

        current_time = datetime.now()
        time_str = current_time.strftime("%Y-%m-%dT%H:%M:%S")
        self.time_string = time_str

    def get_lat_lon_alt(self, lat, lon, alt, time):
        #TODO GPS readings to ADS
        #This was originally for the F' architecture, we will use our own gps
        self.latitude = lat
        self.longitude = lon
        self.altitude = alt
        self.gps_time = time

        

    def static_estimation(self, inside = True):
        #Compute the current sun vector based on the triclops readings
        try:
            vect = compute_vect(self.tri11, self.tri12, self.tri13, self.tri21, self.tri22, self.tri23)
        except Exception as e:
            line = f"Failed to compute Sun Vector {e}\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
        
        if not math.isnan(vect[0]):
            self.sun_vect = vect.flatten()
        else:
            line = f"Nan Vect\n"
            with open(self.filelog, 'a') as file:
                file.write(line) 

        #Compute the magnetic field and sun models
        if inside:
            s = np.array([0.09944678, 0.20356448, 0.97399786])
            s = s / np.linalg.norm(s)
            m = np.array([0.20951717,  0.19442999, -0.95827947])
            m = m / np.linalg.norm(m)
            self.sun_refv = s
            self.mag_ref = m
        else:
            try:
                self.sun_refv, self.mag_ref = get_sun_and_magnetic_field_in_ecef(self.latitude, self.longitude, self.altitude, self.time_string)
            except:
                line = "Failed to compute reference models\n"
                with open(self.filelog, 'a') as file:
                    file.write(line)

        #Run the QUEST static estimation algorithm
        try:
            flat_sun_vect = self.sun_vect
            body_vs = np.vstack((flat_sun_vect, self.mag_vect))
            ref_vs = np.vstack((self.sun_refv, self.mag_ref))
            weights = np.vstack((10,2))
            self.static_q = quest(body_vs, weights, ref_vs)
        except Exception as e:
            line = "Failed to compute static estimates\n"
            with open(self.filelog, 'a') as file:
                file.write(line)

    def kalman_estimation_loop(self):
        try:
            self.get_lat_lon_alt(1,1,1)
        except:
            line = "Failed to get GPS reading\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            # print("Failed to get GPS reading")
        
        try:
            self.getGyroReading()
        except:
            line = "Failed to get Gyro reading\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            # print("Failed to get Gyro reading")

        try:
            self.getMagReading()
        except:
            line = "Failed to get Mag readings\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            # print("Failed to get Mag readings")
        
        try:
            self.getTriclopsReading()
        except Exception as e:
            line = "Failed to get Triclops readings\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            # print(f"Failed to get Triclops readings:{e}")

        try:
            self.get_time_str()
        except:
            line = "Failed to get time string\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            # print("Failed to get Date/Time")
        
        try:
            self.static_estimation(inside=True)
        except:
            line = "Failed Static Estimation\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            # print("Failed to perform Static Estimation")
        
        
        gyro_vect = np.array([self.gyroX, self.gyroY, self.gyroZ])
        try:
            #Kalman predict and update steps
            self.ekf.predict(gyro_vect)
            self.ekf.update(self.static_q.flatten())

            self.dynamic_q = self.ekf.state
        except:
            line = "Failed Kalman Loop\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
        

        rotation = R.from_quat(self.static_q.flatten())
        euler_angles = rotation.as_euler('zyx', degrees=True)
        self.roll, self.pitch, self.yaw = euler_angles

        

        if self.csv_entries >= 1000:
            self.csv_entries = 0
            self.time = time.time() 

            timestamp = datetime.utcfromtimestamp(self.time)
            filecsv = str(self.time) 
            filelog = str(self.time)

            formatted_time = timestamp.strftime('%Y%m%d_%H%M%S')
            folder_name = 'ADS_data'
            folder_name2 = 'ADS_logs'

            if not os.path.exists(folder_name):
                os.makedirs(folder_name)
            if not os.path.exists(folder_name2):
                os.makedirs(folder_name2)

            self.filecsv = formatted_time + '_ADS.csv'
            self.filelog = formatted_time + '_ADSlog.txt'

            self.filecsv = os.path.join(folder_name, self.filecsv)
            self.filelog = os.path.join(folder_name2, self.filelog)

            with open(self.filecsv, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(self.ADS_Header)

        self.time = time.time()
        data = [self.time, self.magX, self.magY, self.magZ, self.gyroX, self.gyroY, self.gyroZ, self.AccelX, self.AccelY, self.AccelZ, self.tri11, self.tri12,
                self.tri13, self.tri21, self.tri22, self.tri23, self.latitude, self.longitude, self.altitude, self.sun_vect,
                self.mag_vect, self.mag_ref, self.sun_refv, self.roll,
                self.pitch, self.yaw, self.static_q.flatten(), self.dynamic_q]

        self.csv_entries += 1

        with open(self.filecsv, 'a', newline='') as file: 
            writer = csv.writer(file)
            writer.writerow(data)

        # Publish dynamic quaternion
        dynamic_quaternion = Quaternion()
        dynamic_quaternion.x = float(self.static_q[0])
        dynamic_quaternion.y = float(self.static_q[1])
        dynamic_quaternion.z = float(self.static_q[2])
        dynamic_quaternion.w = float(self.static_q[3])
        self.publisher_.publish(dynamic_quaternion)

def main(args=None):
    rclpy.init(args=args)
    ads_suite = ADS_Suite()
    rclpy.spin(ads_suite)
    ads_suite.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

