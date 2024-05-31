# Allows current folder to access files in the team-romeo folder
import sys
sys.path.append('/home/fp/team-romeo/ADCS/lib')

# More general Python Libraries
import numpy as np
import time
import csv
import math
from datetime import datetime

# Magnetometer library
import ADCS.lib.pni_rm3100 as pni_rm3100
import magCalParameter

# -------------------- FUNCTIONS --------------------
def countdown():
    print(3)
    time.sleep(1)
    print(2)
    time.sleep(1)
    print(1)
    time.sleep(1)
    print("*clap*")

def correctSensor_v6(x,bxhat,byhat,bzhat):
    # John Springmann | 2/4/11
    a = x[0]
    b = x[1]
    c = x[2]
    x0 = x[3]
    y0 = x[4]
    z0 = x[5]
    rho = x[6]
    phi = x[7]
    lam = x[8]

    bxOut = -(x0 - bxhat)/a
    byOut = -(y0 - byhat + b*bxOut*rho)/b
    bzOut = -(z0 - bzhat +c*lam*bxOut + c*phi*byOut)/c

    dataVec = np.array([bxOut, byOut, bzOut])
    return dataVec
# ---------------------------------------------------


# -------------------- MAIN --------------------
def main():
    mag_addr = 0x21

    #Initialize rm3100 mag, use sampling rate of 37hz by default
    magnetometer = pni_rm3100.PniRm3100()
    magnetometer.assign_device_addr(mag_addr)

    magnetometer.print_status_statements = False
    magnetometer.print_debug_statements = False

    magnetometer.assign_xyz_ccr(x_ccr_in=magnetometer.CcrRegister.CCR_DEFAULT, 
                                    y_ccr_in=magnetometer.CcrRegister.CCR_DEFAULT, 
                                    z_ccr_in=magnetometer.CcrRegister.CCR_DEFAULT)

    magnetometer.assign_tmrc(magnetometer.TmrcRegister.TMRC_37HZ)
    magnetometer.write_ccr()
    magnetometer.write_config()

    # --- Getting Reference Magnetic Field ---
    ref = np.zeros((100,3))
    print("Keep Mag Still | Grabbing Reference:")

    for i in range(100):
        readings = magnetometer.read_meas() # Return the mag readings, each is a raw 3 bytes

        ref[i][0] = readings[0]
        ref[i][1] = readings[1]
        ref[i][2] = readings[2]

        time.sleep(1/37)

    # Make the reference vector the mean of all the samples takes for the current best estimate
    ref = np.mean(ref, axis=0)
    print("Individual Averages: ", ref)
    ref = np.linalg.norm(ref, ord=2)
    print("Total Magnitude Average: ", ref)
    # ----------------------------------------


    # --- Getting a Measurement Sphere ---
    # Define the number of samples to calibrate from, more is better
    samples = 100
    mags = np.zeros((samples,3))

    print("Get Ready, Measuring in:")
    countdown()

    # Loop over the magnetometer for the desired number of samples
    for i in range(samples):
        readings = magnetometer.read_meas() # Return the mag readings, each is a raw 3 bytes

        mags[i][0] = readings[0]
        mags[i][1] = readings[1]
        mags[i][2] = readings[2]

        time.sleep(1/37)

    # Save the array to a CSV file | Does not number the columns
    column_names = ["mag_x", "mag_y", "mag_z"]
    np.savetxt("mag_raw.csv", mags, delimiter=",", header=",".join(column_names), comments="", fmt="%f")
    # --------------------------------------


    # --- Calibrating the Measurements ---
    # Find the calibration paramenters, continues if they already exist
    try:
        magCalParameter.main('mag_raw.csv',ref)
    except:
        print("Continuing...")

    # Pulls out the parameters and uses them to adjust the sensor
    params = np.zeros((9,1))
    i = 0
    with open('calibrated_magnetometer_params/mag_raw_calibrationParameters.csv', 'r') as file:
        # Read through the parameters
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            params[i] = row[1]
            i += 1

    # Calibrate the data that was taken
    calib_mag = correctSensor_v6(params,mags[:,0],mags[:,1],mags[:,2])
    calib_mag = calib_mag.T

    # Save the array to a CSV file | Does not number the columns
    column_names = ["mag_x", "mag_y", "mag_z"]
    np.savetxt("mag_calib.csv", calib_mag, delimiter=",", header=",".join(column_names), comments="", fmt="%f")

    # Print out the desired parameters
    print("Calibration Parameters: ")
    print(params)
    # --------------------------------------
# ---------------------------------------