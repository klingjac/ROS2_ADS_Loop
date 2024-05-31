# Allows current folder to access files in the team-romeo folder
import sys
sys.path.append('/home/fp/team-romeo/ADCS/lib')

# More general Python Libraries
import numpy as np
import time
from datetime import datetime

# imu library
from icm20948_lib import ICM20948
from icm20948_lib import I2C_ADDR_ALT

CAL = np.array([-0.84788618, -0.13729675, -0.22345528])

# IMU_addr = 0x69

#Initialize gyro as +- 4g and +- 2000 dps
imu_gyro = ICM20948(I2C_ADDR_ALT)
imu_gyro.set_accelerometer_full_scale(4)
imu_gyro.set_gyro_full_scale(2000)

#bytes and reading for the gyro
def getGyroReading() -> bytes:
    #data = imu.read_bytes(ICM20948_ACCEL_XOUT_H + 6, 12) #Return the raw gyro readings, each is a raw 2 bytes
    ax, ay, ag, gx, gy, gz = imu_gyro.read_accelerometer_gyro_data()
    gyroX = gx
    gyroY = gy
    gyroZ = gz
    data = np.array([gyroX, gyroY, gyroZ])

    return data

def sample_axis(dc,samples,iter=0):
    for i in range(samples):
        ind = samples*iter + i
        dc[ind] = getGyroReading()
        time.sleep(1/250)

def fancyDC(samples):
    dc = np.zeros((samples*3,3))
    for n in range(3):
        print("Axis: ", n+1)
        time.sleep(5)
        print("Starting...")
        sample_axis(dc,samples,n)
    dc = np.mean(dc, axis=0)
    print("Individual Averages: ", dc)

def simpleDC(samples):
    dc = np.zeros((samples,3))
    sample_axis(dc,samples)
    dc = np.mean(dc, axis=0)
    print("Individual Averages: ", dc)

def angleEst():
    print("Go!")
    rate = 37
    t = 5
    samples = t*rate
    dps = np.zeros((samples,3))
    for i in range(samples):
        start_time = time.time()
        dps[i] = getGyroReading() - CAL
        end_time = time.time()
        dt = end_time - start_time
        sleeper = 1/rate - dt
        time.sleep(sleeper)
    turn = dps/rate
    turn = np.sum(turn, axis=0)
    print(turn)

simpleDC(3000)