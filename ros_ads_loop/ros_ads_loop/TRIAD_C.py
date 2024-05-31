import numpy as np
from scipy.spatial.transform import Rotation as R
import math

#INPUTS
# reference vectors (linear. indep.)
# R_1, R_2

# measured dir. of ref. vectors in BFF
# r_1, r_2


# CALCULATIONS
# using unit vectors to account for noisy r_1 and r_2 values
def extract_euler_angles_from_matrix(BN):
    # Assuming BN is a 3x3 rotation matrix and the sequence is XYZ
    #print("BN:", BN)
    theta = np.arctan2(-BN[2, 0], np.sqrt(BN[2, 1]**2 + BN[2, 2]**2))
    if np.abs(np.cos(theta)) > np.finfo(float).eps:  # Avoid division by zero or gimbal lock
        psi = np.arctan2(BN[1, 0] / np.cos(theta), BN[0, 0] / np.cos(theta))
        phi = np.arctan2(BN[2, 1] / np.cos(theta), BN[2, 2] / np.cos(theta))
    else:
        # Handle gimbal lock, assume phi = 0
        psi = np.arctan2(-BN[0, 1], BN[1, 1])
        phi = 0

    return np.rad2deg(phi), np.rad2deg(theta), np.rad2deg(psi)  # Convert from radians to degrees


def compute_triad(sun_ref, mag_ref, sun_vec, mag_vec):
    # Normalize the input vectors
    sun_vec = sun_vec.flatten()
    s_hat_ref = sun_ref / np.linalg.norm(sun_ref)
    m_hat_ref = mag_ref / np.linalg.norm(mag_ref)
    s_hat_vec = sun_vec / np.linalg.norm(sun_vec)
    m_hat_vec = mag_vec / np.linalg.norm(mag_vec)

    # print("1 ", s_hat_ref)
    # print("2 ", m_hat_ref)
    # print("3 ", s_hat_vec)
    # print("4 ", m_hat_vec)

    # Compute the secondary reference vectors by taking the cross product and normalize them
    t2_hat_ref = np.cross(s_hat_ref, m_hat_ref)
    t2_hat_ref /= np.linalg.norm(t2_hat_ref)
    t2_hat_vec = np.cross(s_hat_vec, m_hat_vec)
    t2_hat_vec /= np.linalg.norm(t2_hat_vec)

    # Compute the tertiary reference vectors by taking the cross product
    t3_hat_ref = np.cross(s_hat_ref, t2_hat_ref)
    t3_hat_vec = np.cross(s_hat_vec, t2_hat_vec)

    # Construct the reference and body frame matrices
    NT = np.column_stack((s_hat_ref, t2_hat_ref, t3_hat_ref))
    BT = np.column_stack((s_hat_vec, t2_hat_vec, t3_hat_vec))

    # Compute the rotation matrix from body to reference frame
    # print("BT: ", BT)
    # print("NT: ", NT.T)
    BN = BT @ NT.T
    
    # Extract Euler angles (assuming 'xyz' convention for this example)
    r = R.from_matrix(BN)
    eulers = r.as_euler('xyz', degrees=True)

    roll_degrees = (eulers[0])
    pitch_degrees = (eulers[1])
    yaw_degrees = (eulers[2])

    # Adjust pitch to be within -180 to 180 degrees
    # Note: This step might be redundant if pitch is already in this range
    #pitch_degrees = (pitch_degrees + 180) % 360 - 180

    # Adjust yaw to be within 0 to 360 degrees
    #yaw_degrees = yaw_degrees % 360
    
    return roll_degrees, pitch_degrees, yaw_degrees
