import ephem
import numpy as np
import pymap3d as pm
import ahrs
from datetime import datetime

def magnetic_field_ned_to_ecef(lat, lon, alt, time):
    """
    Computes the magnetic field vector in the ECEF frame using the WMM model from the AHRS library.
    
    Parameters:
    lat (float): Latitude in degrees.
    lon (float): Longitude in degrees.
    alt (float): Altitude in m.
    time (datetime): Datetime object representing the current time.
    
    Returns:
    numpy.ndarray: Magnetic field vector in the ECEF frame.
    """
    dt = datetime.fromisoformat(str(time))

    # Compute the magnetic field in NED frame using AHRS library
    wmm = ahrs.utils.WMM()

    #Note: AHRS utilizes km for altitude

    wmm.magnetic_field(lat, lon, alt/1000, dt.date())
    mag_ned = [wmm.X, wmm.Y, wmm.Z]
    
    # Convert NED to ECEF
    mag_ecef = pm.ned2ecef(mag_ned[0], mag_ned[1], mag_ned[2], lat, lon, alt)
    mag_ref = np.array(mag_ecef)

    mag_ref = mag_ref / np.linalg.norm(mag_ref)

    return mag_ref

def get_sun_position_ecef(lat, lon, alt, time_str):
    """
    Compute the Sun's position as a unit vector in the ECEF frame using PyEphem for a given time.
    
    Parameters:
    lat (float): Latitude in degrees.
    lon (float): Longitude in degrees.
    alt (float): Altitude in meters.
    time_str (str): Time in ISO 8601 string format.
    
    Returns:
    numpy.ndarray: Sun's position as a unit vector in the ECEF frame.
    """
    # Convert time to PyEphem readable format
    time_strm = time_str.replace("T", " ")  # Replace 'T' with a space
    observer = ephem.Observer()
    observer.lat, observer.lon, observer.elevation = str(lat), str(lon), alt
    observer.date = ephem.date(time_strm)
    
    # Get Sun's position
    sun = ephem.Sun(observer)
    az, alt = sun.az, sun.alt  # Azimuth and altitude

    # Convert from spherical (Azimuth, Altitude) to ECEF using pymap3d
    x, y, z = pm.enu2ecef(np.cos(alt) * np.sin(az), np.cos(alt) * np.cos(az), np.sin(alt), lat, lon, alt)
    
    # Normalize the vector
    sun_position = np.array([x, y, z])
    sun_unit_vector = sun_position / np.linalg.norm(sun_position)
    
    return sun_unit_vector

def get_sun_and_magnetic_field_in_ecef(lat, lon, alt, time_str):
    """
    Compute the Sun's position and the magnetic field vector in the ECEF frame
    for a given latitude, longitude, altitude, and time using PyEphem for the Sun's position.
    
    Parameters:
    lat (float): Latitude in degrees.
    lon (float): Longitude in degrees.
    alt (float): Altitude in meters.
    time_str (str): Time in ISO 8601 string format.
    
    Returns:
    tuple: Tuple containing the Sun's position in the ECEF frame and the magnetic field vector in the ECEF frame.
    """
    datetime_time = datetime.fromisoformat(time_str)
    
    # Calculate the Sun's position in the ECEF frame
    sun_position_ecef = get_sun_position_ecef(lat, lon, alt, time_str)
    
    # Calculate the magnetic field vector in the ECEF frame
    magnetic_field_vector_ecef = magnetic_field_ned_to_ecef(lat, lon, alt, datetime_time)
    
    return sun_position_ecef, magnetic_field_vector_ecef

if __name__ == "__main__":
    lat, lon, alt = 40.0, -105.0, 1000.0  # Altitude in meters for consistency
    time_str = "2023-01-01T12:00:00"
    sun_position_ecef, magnetic_field_vector_ecef = get_sun_and_magnetic_field_in_ecef(lat, lon, alt, time_str)
    print(f"Sun Position in ECEF: {sun_position_ecef}")
    print(f"Magnetic Field Vector in ECEF: {magnetic_field_vector_ecef}")
