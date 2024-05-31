import ADCS.lib.pni_rm3100
from ADCS.lib.correctSensor_v6 import correctSensor_v6
import time
import threading
import math
from datetime import datetime
import csv
import os

from ADCS.lib.icm20948_lib import ICM20948
from ADCS.lib.icm20948_lib import I2C_ADDR_ALT
from ADCS.lib.icm20948_lib import AK09916_HXL
from ADCS.lib.icm20948_lib import AK09916_CNTL2
from ADCS.lib.icm20948_lib import ICM20948_ACCEL_XOUT_H

from ADCS.lib.AD7994 import AD7994 
from ADCS.lib.pni_rm3100 import PniRm3100

from ADCS.src.sun_vect import compute_vect
from ADCS.src.QUEST import quest
from ADCS.src.models import get_sun_and_magnetic_field_in_ecef
from ADCS.src.EKFQ import QuaternionKalmanFilter
from ADCS.src.rtc import gps_utc2utc

from scipy.spatial.transform import Rotation as R
import numpy as np
import struct

class ADS_Sensors():

    #Sensor Objects
    magnetometer = None
    imu_gyro = None
    triclops = None
    triclops2 = None

    #Kalman Filter Objec
    ekf = QuaternionKalmanFilter()

    mag_addr = 0x21

    #Private State Variables For ADS Estimation
    gyroX = 0
    gyroY = 0
    gyroZ = 0

    gyro_dc = np.array([-0.54597561,  0.16878049, -0.45178862])

    magX = 0
    magY = 0
    magZ = 0

    calMagX = 0
    calMagY = 0
    calMagZ = 0

    mag_params = np.array([1.1896623460246800, 1.2096018706546000, 1.1723232183557600, 
                           0.09204406440799900, -1.0490628812756700, -3.0031107843628300,
                           0.03164687777013330, 0.030998399519443400, 0.02189148151949750])

    tri1 = 0
    tri2 = 0
    tri3 = 0

    tri21 = 0
    tri22 = 0
    tri23 = 0

    latitude = 42.3
    longitude = -87.1
    altitude = 0.3
    gps_time = 0

    time_string = 0
    time = 0

    sun_vect = np.array([0,0,1])
    mag_vect = np.array([0,0,1])

    mag_ref = 0
    sun_refv = 0

    static_roll = 0
    static_pitch = 0
    static_yaw = 0

    roll = 0
    pitch = 0
    yaw = 0

    csv_entries = 0

    filecsv = None
    filelog = None

    static_q = np.array([0, 0, 0, 1])
    dynamic_q = np.array([0, 0, 0, 1])

    ADS_Header = ['Time', 'MagX', 'MagY', 'MagZ', 'GyroX', 'GyroY', 'GyroZ', 'Tri11', 'Tri12', 'Tri13',
                  'Tri21', 'Tri22', 'Tri23', 'Lat', 'Lon', 'Alt', 'SunVect', 'MagVect', 'MagRef', 'SunRef', 
                  'Roll', 'Pitch', 'Yaw', 'static_q', 'dynamic_q']

    def __init__(self, _RTC=None):
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
        self.triclops = AD7994(address=0x20)
        self.triclops2 = AD7994(address=0x24)
        self.sun_ref = 3
        self.MaxCounts = 1023
        self.RTC = _RTC

        # try:
        #     self.time = self.RTC.getTime()
        #     print("RTC")
        # except Exception as e:
        #     print(f"Error: {e}")
        #     self.time = time.time()
        # arr = self.parse_datetime(rtc.getFullTime)
        # dt = datetime(arr[2], arr[1], arr[0], arr[3], arr[4], arr[5])  # Replace with desired date and time
        # subprocess.call(['sudo', 'date', '-s', '{:}'.format(dt.strftime('%Y/%m/%d %H:%M:%S'))])

        self.time = time.time() #TODO: UNCOMMENT ABOVE

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

        main = threading.Thread(target=self.kalman_estimation_loop)
        main.start()

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

    def getTriclopsReading(self) -> bytes:
        data = self.triclops.get_data() 
        
        byte_convert = [point * self.sun_ref/self.MaxCounts for point in data]
        self.tri1 = byte_convert[0]
        self.tri2 = byte_convert[1]
        self.tri3 = byte_convert[2]
        

        data2 = self.triclops2.get_data()

        byte_string = b''.join([i.to_bytes(2, byteorder='big', signed=False) for i in data])
        byte_string2 = b''.join([i.to_bytes(2, byteorder='big', signed=False) for i in data2])

        first_three_bytes = byte_string[:6]
        first_three_bytes2 = byte_string2[:6]
        bs = first_three_bytes + first_three_bytes2

        
        byte_convert = [point * self.sun_ref/self.MaxCounts for point in data2]
    
        self.tri21 = byte_convert[0]
        self.tri22 = byte_convert[1]
        self.tri23 = byte_convert[2]
        
        return bs

    def getMagReading(self) -> bytes:
        readings = self.magnetometer.read_meas() 

        # Relative magnetometer readings
        magX = readings[0]
        magY = readings[1]
        magZ = readings[2]

        # MC10 Orientation mag
        self.magX = magY
        self.magY = magX
        self.magZ = magZ

        # UNCOMMENT FOR CALIBRATED VERSION
        mags = correctSensor_v6(self.mag_params, magX, magY, magZ)
        magX = mags[0]
        magY = mags[1]
        magZ = mags[2]

        magXb = readings[3]
        magYb = readings[4]
        magZb = readings[5]

        self.mag_vect = np.array([magY, magX, -magZ])
        self.mag_vect = self.mag_vect / np.linalg.norm(self.mag_vect)

        magXb = int(self.magX*100)
        magXb = magXb.to_bytes(4, byteorder='big', signed=True)
        magYb = int(self.magY*100)
        magYb = magYb.to_bytes(4, byteorder='big', signed=True)
        magZb = int(self.magZ*100)
        magZb = magZb.to_bytes(4, byteorder='big', signed=True)
        bts = magXb + magYb + magZb

        return bts

    def getGyroReading(self) -> bytes:
        #data = self.imu.read_bytes(ICM20948_ACCEL_XOUT_H + 6, 12) #Return the raw gyro readings, each is a raw 2 bytes
        ax, ay, ag, gx, gy, gz = self.imu_gyro.read_accelerometer_gyro_data()
        
        # UNCOMMMENT FOR CALIBRATED VERSION
        gx -= self.gyro_dc[0]
        gy -= self.gyro_dc[1]
        gz -= self.gyro_dc[2]

        # MC10 relative gyro
        self.gyroX = gy
        self.gyroY = gx
        self.gyroZ = -1*gz

        # Gyro to bits
        gyroXe = int(gx*100)
        gyroXe = gyroXe.to_bytes(4, byteorder='big', signed=True)
        gyroYe = int(gy*100)
        gyroYe = gyroYe.to_bytes(4, byteorder='big', signed=True)
        gyroZe = int(gz*100)
        gyroZe = gyroZe.to_bytes(4, byteorder='big', signed=True)
        bts = gyroXe + gyroYe + gyroZe

        return bts
    
    #Currently this just utilizes system time
    def get_time_str(self):
        #TODO: Pass time and data to ADS at some point

        current_time = datetime.now()

        time_str = current_time.strftime("%Y-%m-%dT%H:%M:%S")

        self.time_string = time_str

    def get_lat_lon_alt(self, lat, lon, alt, time):
        #TODO GPS readings to ADS
        self.latitude = lat
        self.longitude = lon
        self.altitude = alt
        self.gps_time = time

    #def start_kalman



    def static_estimation(self, inside = False):
        #Compute the current sun vector based on the triclops readings
        try:
            vect = compute_vect(self.tri1, self.tri2, self.tri3, self.tri21, self.tri22, self.tri23)
        except:
            line = "Failed to compute Sun Vector\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            # print("Failed to compute Sun Vector")
        
        if not math.isnan(vect[0]):
            self.sun_vect = vect

        #Compute the magnetic field and sun models
        if inside:
            s = np.array([-0.85, -0.172, 0.4912])
            s = s / np.linalg.norm(s)
            m = np.array([-0.1757, 0.667, -0.724])
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
                # print("Failed to compute reference models")

        #Run the Triad Static Estimation
        try:
            flat_sun_vect = self.sun_vect.flatten()
            body_vs = np.vstack((flat_sun_vect, self.mag_vect))
            ref_vs = np.vstack((self.sun_refv, self.mag_ref))
            weights = np.vstack((10,2))
            self.static_q = quest(body_vs, weights, ref_vs)
        except Exception as e:
            line = "Failed to compute static estimates\n"
            with open(self.filelog, 'a') as file:
                file.write(line)
            #print(f"Failed to compute static estimates:{e}")


    def kalman_estimation_loop(self):
        
        while True:
            # try:
            #     self.get_lat_lon_alt(1,1,1)
            # except:
            #     line = "Failed to get GPS reading\n"
            #     with open(self.filelog, 'a') as file:
            #         file.write(line)
            #     # print("Failed to get GPS reading")
            
            # try:
            #     self.getGyroReading()
            # except:
            #     line = "Failed to get Gyro reading\n"
            #     with open(self.filelog, 'a') as file:
            #         file.write(line)
            #     # print("Failed to get Gyro reading")

            # try:
            #     self.getMagReading()
            # except:
            #     line = "Failed to get Mag readings\n"
            #     with open(self.filelog, 'a') as file:
            #         file.write(line)
            #     # print("Failed to get Mag readings")
            
            # try:
            #     self.getTriclopsReading()
            # except Exception as e:
            #     line = "Failed to get Triclops readings\n"
            #     with open(self.filelog, 'a') as file:
            #         file.write(line)
            #     # print(f"Failed to get Triclops readings:{e}")

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
            # print(f"Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}")

            rotation = R.from_quat(self.static_q.flatten())
            euler_angles = rotation.as_euler('zyx', degrees=True)
            self.roll, self.pitch, self.yaw = euler_angles

            # ADS_Header = ['Time', 'MagX', 'MagY', 'MagZ', 'GyroX', 'GyroY', 'GyroZ', 'Tri11', 'Tri12', 'Tri13',
            #       'Tri21', 'Tri22', 'Tri23', 'Lat', 'Lon', 'Alt', 'SunVect', 'MagVect', 'MagRef', 'SunRef', 
            #       'Roll', 'Pitch', 'Yaw']

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
            data = [self.time, self.magX, self.magY, self.magZ, self.gyroX, self.gyroY, self.gyroZ, self.tri1, self.tri2,
                    self.tri3, self.tri21, self.tri22, self.tri23, self.latitude, self.longitude, self.altitude, self.sun_vect,
                    self.mag_vect, self.mag_ref, self.sun_refv, self.roll,
                    self.pitch, self.yaw, self.static_q.flatten(), self.dynamic_q]

            self.csv_entries += 1

            with open(self.filecsv, 'a', newline='') as file: 
                writer = csv.writer(file)
                writer.writerow(data)

            time.sleep(1/37)
                

    #Use this function to extract the Euler angles in bytes
    #All values are signed and returned little endian

    def getKalmanEstimateRoll(self) -> bytes:
        rollb = self.roll
        rollb = int(rollb * 100)
        rollb = rollb.to_bytes(2, byteorder='big', signed=True)
        
        return rollb

    def getKalmanEstimatePitch(self) -> bytes:
        pitchb = self.pitch
        pitchb = int(pitchb * 100)
        pitchb = pitchb.to_bytes(2, byteorder='big', signed=True)

        return pitchb

    def getKalmanEstimateYaw(self) -> bytes:
        yawb = self.yaw
        yawb = int(yawb * 100)
        yawb = yawb.to_bytes(2, byteorder='big', signed=True)

        return yawb

    
    

