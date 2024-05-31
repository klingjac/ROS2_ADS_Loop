import os
import sys
import pandas as pd
import numpy as np


def extractParameters_v6a(x,bxhat,byhat,bzhat,bmag,all_curr):
    # John Springmann 
    # 2/4/11
    #
    # This function uses non-linear least-squares to extract the calibration
    # parameters utilizing a known but time-varying magnetic field magnitude.
    #
    #   2/5/11 v3
    #       Add SP currents & scaling factors
    #   4/4/11 v4
    #       Add telemetry point for beacons (EPS Vbatt current)
    #   5/15/15 v5 Editor: Nathanael England
    #       Cleaning up syntax of the code and generalizing current portions
    #   5/22/15 v6 Editor: Nathanael England
    #       Generalizing code for arbitrary current inputs
    #       Removed initial guess as input. Monte Carlo simulation in
    #       Springmann's paper shows initial estimate isn't important for
    #       convergence
    #   1/25/21
    #       Diego Suazo De la Rosa converted to Python

    # inputs:
    #   bxhat: x-component of the sensor reading
    #   byhat: y-component of the sensor reading
    #   bzhat: z-component of the sennsor reading
    #   bmag: the magnitude of the mag field used for calibration. This is a
    #           vector the same size as the measurements.
    #   all_curr: Matrix of currents on board. NxM matrix for N data points and
    #   M current sources on board

    eps = 1

    # the "measurements" is the array of field magnitude squared
    meas = np.square(bmag)

    J = np.array([0])    # need to store the loss function values
    ctr = 1    # counter, start at 2

    cts_max = 50;    # upper limit on the number of iterations

    data_num, num_curr_sources = all_curr.shape

    while eps > 1e-14:
        
        dfdx = np.zeros((len(bxhat),len(x)))
        f = np.zeros((len(bxhat),1));
        a = x[0]
        b = x[1]
        c = x[2]
        x0 = x[3]
        y0 = x[4]
        z0 = x[5]
        rho = x[6]
        phi = x[7]
        lam = x[8]

        for v in range(len(bxhat)):
            sIx = 0
            sIy = 0
            sIz = 0
            # a function for each measurement

            Bxhat = bxhat[v]
            Byhat = byhat[v]
            Bzhat = bzhat[v]

            # Sum the current effects
            for j in range(num_curr_sources):
                sIx = sIx + x[j+9]*all_curr[v,j]
                sIy = sIy + x[j+9+num_curr_sources]*all_curr[v,j]
                sIz = sIz + x[j+9+2*num_curr_sources]*all_curr[v,j]
            
            # Generate B^2 = Bx^2 + By^2 + Bz^2 by inverting equations below
            # Bxhat = a*Bx + x0 + sIx
            # Byhat = b*(By + rho*Bx) + y0 + sIy
            # Bzhat = c*(lam*Bx + phi*By + Bz) + z0 + sIz
            Bx = -(x0 - Bxhat + sIx)/a
            By = -(y0 - Byhat + sIy + b*Bx*rho)/b
            Bz = -(z0 - Bzhat + sIz + c*lam*Bx + c*phi*By)/c
            f[v] = np.square(Bx) + np.square(By) + np.square(Bz)

            # Generate Jacobian
            dfdx[v,0] = 2*Bx*(-Bx/a)+2*By*(Bx*rho/a)+2*Bz*(Bx/a)*(lam-phi*rho)
            dBy_db = (By*(-b)-Bx*rho*b)/np.square(b)
            dBz_db = -phi*dBy_db
            dfdx[v,1] = 2*Bx*(0) + 2*By*dBy_db + 2*Bz*dBz_db
            dfdx[v,2] = 2*Bx*(0)+2*By*(0)+2*Bz*(Bz*(-c)-c*(lam*Bx+phi*By))/np.square(c)
            dfdx[v,3] = 2*Bx*(-1/a)+2*By*(rho/a)+2*Bz*(1/a)*(lam-phi*rho)
            dfdx[v,4] = 2*Bx*(0)+2*By*(-1/b)+2*Bz*(phi/b)
            dfdx[v,5] = 2*Bx*(0)+2*By*(0)+2*Bz*(-1/c)
            dfdx[v,6] = 2*Bx*(0)+2*By*(-Bx)+2*Bz*(phi*Bx)
            dfdx[v,7] = 2*Bx*(0)+2*By*(0)+2*Bz*(-By)
            dfdx[v,8] = 2*Bx*(0)+2*By*(0)+2*Bz*(-Bx)
            for w in range(num_curr_sources):
                I_curr = all_curr(v,w)
                k1 = w + 9
                k2 = w + 9 + num_curr_sources
                k3 = w+ 9 + num_curr_sources*2
                dfdx[v,k1]= 2*Bx*(-I_curr/a)+2*By*(rho*I_curr/a)+2*Bz*(I_curr/a)*(lam-phi*rho)
                dfdx[v,k2]= 2*Bx*(0)+2*By*(-I_curr/b)+2*Bz*(phi*I_curr/b)
                dfdx[v,k3]= 2*Bx*(0)+2*By*(0)+2*Bz*(-I_curr/c)
                
        deltay = meas - f # measurements are the mag field magnitude

        np.mean(deltay)
        deltaycon = np.atleast_2d(deltay).T.conj()
        Jhere = deltaycon.dot(deltay)

        J = np.append([J], [Jhere[0]])

        H = dfdx
        Hcon = np.atleast_2d(H).T.conj()

        alpha = 0.01 # regularization parameter

        deltax_1 = np.linalg.matrix_power((Hcon.dot(H) + alpha*np.eye(9)),-1)
        deltax_2 = Hcon.dot(deltay)
        deltax = deltax_1.dot(deltax_2)
        eps = abs(J[ctr] - J[ctr-1])/J[ctr]
        x = x + deltax
        
        # check that the scaling factors are positive
        x[0] = abs(x[0])
        x[1] = abs(x[1])
        x[2] = abs(x[2])
        ctr = ctr+1
        
        if ctr > cts_max:
            print('Max iterations reached')
            break
        
    cal_params = x
    return cal_params

        
def example_calibration(filename,magRefNorm):
    '''
    Diego Suazo De la Rosa
    This is a transcription of Prince's Matlab code to Python:

    Prince Kuevor
    January 18, 2022

    This script is meant to be an example of how to use Springman's
    Magnetometer calibration code. 

    I've included the needed functions here for convenience, but know that
    they also exist in the MXL SVN repos at...
    SVN/grifex/Operations/Data_Analysis/Magnetometer/functions/

    In addition, these funcitons exist in Prince's private research code Git
    repo. 

    Please contact Prince Kuevor (kuevpr@umich.edu) or Diego Suazo (dsuazo@umich.edu)
    if you have any trouble finding the necesasary code for this example script. 
    '''

    # --------------- Optional Parameters --------------- #
    # Options for plotting and/or saving parameters

    # 'true' if you want the magnetometer calibration parameters saved to a
    # file. Ensure 'outfile_folder' and 'outfile_suffix' are set to desired
    # values if you want this to be 'true'.
    # 'false' otherwise
    save_params = 'true'

    # Apply median filter to magnetometer data in body frame
    # This is to help smooth some noise of the sensor. This parameter
    # can/should be adjusted based on your needs and your sensor. Other
    # smoothing techniques (moving average filter, low-pass filter, etc...) can
    # be used as well. 
    median_filter_window_rm3100 = 4

    # --------------- Reference Magnetic Field Norm  --------------- #
    # Constant magnetic field reference value needed to calibrate magnetometer
    # data

    # When using this script to calibrate, you must update this value with the
    # actualy ambient magnetic field _magnitude_ at the location you are
    # calibrating. The function depends on this and is inputted beforehand.

    # Note: calibrating indoors is difficult because the magnitude of the
    # ambient magnetic field is very sensitive to small movements

    # --------------- Select Filenames --------------- #

    # Output File Folder
    outfile_folder = "calibrated_magnetometer_params/"

    # Suffix added to calibrated magnetometer data
    outfile_suffix = "_calibrationParameters.csv"

    ## Make directory for outfile so rest of things work smoothly
    if not (os.path.isdir(outfile_folder)):
        os.makedirs(outfile_folder)

    # --------------- Iterate Over Each Flight Number --------------- #
    # Construct input and output filenames
    infile = filename

    outfile = outfile_folder + filename[0:len(filename)-4] + outfile_suffix

    # If the output file already exists, move on
    if os.path.exists(outfile):
        try:
            print("File Already Exists -> %s" % outfile)
            sys.exit("Stopping calibration script to avoid needless computation")
        except:
            sys.exit("Unable to make folder %s" % outfile_folder)

    # ----------------------------------#
    #       Read in Logfile Data        #
    # ----------------------------------#

    # Read data from 'infile'
    try:
        data = pd.read_csv(infile)
    except:
        # Can't find a file? Skip it and move on
        print("Could not find %s", infile)
        sys.exit("Update 'infile', 'infile_folder', 'filename', and 'file_extension' variables then run again")


    # --------------- Calibrate Mag --------------- #
    # Select data from potentially huge data struct and convert to array
    # Indeces we're selecting here are 'mag_x', 'mag_y', and 'mag_z'
    # and should be named as such in your csv file. You can alternatively
    # just change the name for the desired columns below.
    mag_data = pd.DataFrame(data,columns = ["mag_x", "mag_y", "mag_z"])
    
    # Make reference scalar into a vector
    magRefNorm_vec = magRefNorm * np.ones((mag_data.shape[0],1))
    
    # Initial guess for (scale factors, bias, and non-orthogonality angles)
    x0 = np.array([ [1], [1], [1], [0], [0], [0], [0], [0], [0]])


    # Apply moving median filter with window size of 'median_filter_window_rm3100'

    lb = median_filter_window_rm3100/2     # lowerbound
    ub = median_filter_window_rm3100 - lb  # upperbound
    window = int(1+(lb+ub)/2)
    mag_data = np.array([mag_data.iloc[:,0].rolling(window,min_periods = 1).median(),
                        mag_data.iloc[:,1].rolling(window,min_periods = 1).median(),
                        mag_data.iloc[:,2].rolling(window,min_periods = 1).median()])

    # Corrections for high-current-carrying wires near magnetometer
    # This is for time-varying effects which I (Prince) do not account for in
    # my applications.
    allCurrents = np.array([[]])

    # Calibration parameters (Body frame)
    # This is one of Springman's functions that actually does the magnetomer
    # calibration.

    calParams_B_rm3100 = extractParameters_v6a(x0,mag_data[0,:],mag_data[1,:],mag_data[2,:],
                                               magRefNorm_vec,allCurrents)


    # ----------------------------------#
    #          Save Parameters          #
    # ----------------------------------#
    if save_params == "true":
        try:
            params= pd.DataFrame(calParams_B_rm3100)
            params.to_csv(outfile)
            print("Finished processing -> %s" % outfile)
        except:
            sys.exit("Unable to write -> %s" % outfile)

    
def main(filename, magRefNorm):
    example_calibration(filename, magRefNorm)
