import numpy as np

def correctSensor_v6(x,bxhat,byhat,bzhat):
    # John Springmann
    # 2/4/11
    #
    # Correct the senor measurements based on the calibration parameters
    #
    #   2/5/11 v3
    #       Uses model that includes SP currents
    #   4/4/11 v4
    #       Uses 5th current telem point to try to get beacons out
    #   5/21/15 v5
    #       Generalizing to accept output from extractParameters_v6
    #   1/25/22 v6
    #       Transcribed to Python from MATLAB by Diego Suazo De la Rosa
    #   2/14/22 v6.2
    #       Removed current calculations since these aren't used by the
    #       parameter estimation side of things (yet?).

    # inputs:
    #   x: vector of calibration parameters. Order is 
    #       [a b c x0 y0 z0 rho phi lam]
    #   bxhat: x-component of raw measurements (vector)
    #   byhat: y-component of raw measurements (vector)
    #   bzhat: z-component of raw measurements (vector)

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