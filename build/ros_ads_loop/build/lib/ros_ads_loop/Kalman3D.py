import numpy as np

class Kalman():
    #State vector: roll, pitch, yaw
    state = np.array([0, 0, 0])

    dt = 0.1

    #F-matrix is just one for the static estimate
    F = np.eye(3)

    #Gyro-control input for this approach
    B = np.eye(3) * dt

    #Measurement model matrix for heading
    Hm = np.eye(3)

    P = np.eye(3) * 1
    Q = np.eye(3) * 0.1

    R_heading = np.diag([0.5, 0.5, 0.5]) #Guess values currently

    def wrap_pitch_roll(self, angle_degrees):
        """Wrap the yaw angle to [0, 360] degrees"""
        angle_degrees = angle_degrees % 360
        if angle_degrees > 180:
            angle_degrees -= 360
        return angle_degrees

    def wrap_yaw_roll(self, angle_degrees):
        """Wrap the pitch or roll angle to [-180, 180] degrees"""
        angle_degrees = angle_degrees % 360
        if angle_degrees > 180:
            angle_degrees -= 360
        return angle_degrees

    def body_rates_to_euler_rates(self, phi, theta, p, q, r):
        """
        Convert body rates (p, q, r) to Euler rates (roll rate, pitch rate, yaw rate).
        
        Parameters:
        phi (float): Roll angle in radians.
        theta (float): Pitch angle in radians.
        p (float): Body rate around the x-axis (roll rate) in radians per second.
        q (float): Body rate around the y-axis (pitch rate) in radians per second.
        r (float): Body rate around the z-axis (yaw rate) in radians per second.
        
        Returns:
        np.ndarray: A 3-element array containing [roll rate, pitch rate, yaw rate] in radians per second.
        """
        # Transformation matrix from body rates to Euler rates
        transformation_matrix = np.array([
            [1, np.sin(np.radians(phi)) * np.tan(np.radians(theta)), np.cos(np.radians(phi)) * np.tan(np.radians(theta))],
            [0, np.cos(np.radians(phi)), -np.sin(np.radians(phi))],
            [0, np.sin(np.radians(phi)) / np.cos(np.radians(theta)), np.cos(np.radians(phi)) / np.cos(np.radians(theta))]
        ])
        
        # Body rates vector
        body_rates = np.array([p, q, r])
        
        # Calculate Euler rates
        euler_rates = np.dot(transformation_matrix, body_rates)
        #print(body_rates)
        #print(euler_rates)
        return euler_rates


    def kalman_run(self, gyro_vect, static_vect):
        u = gyro_vect

        x_predict = self.F @ self.state + self.B @ self.body_rates_to_euler_rates(self.state[0], self.state[1], u[0], u[1], u[2])
        
        
        x_predict[0] = self.wrap_yaw_roll(x_predict[0])
        x_predict[1] = self.wrap_pitch_roll(x_predict[1])
        x_predict[2] = self.wrap_yaw_roll(x_predict[2])
            

        p_predict = self.F @ self.P @ self.F.T + self.Q

        self.kalman_update(static_vect, x_predict, p_predict)

        return self.state[0], self.state[1], self.state[2]

    def kalman_update(self, static_vect, x_predict, p_predict):
        K_heading = p_predict @ self.Hm.T @ np.linalg.inv(self.Hm @ p_predict @ self.Hm.T + self.R_heading)
        # print("This hm: ", self.Hm @ x_predict)
        # print("This static: ", static_vect)
        x_heading_up = x_predict + K_heading @ (static_vect - self.Hm @ x_predict)

        P_heading_up = p_predict - K_heading @ self.Hm @ p_predict

        self.P = P_heading_up
        self.state[0] = self.wrap_yaw_roll(x_heading_up[0])
        self.state[1] = self.wrap_pitch_roll(x_heading_up[1])
        self.state[2] = self.wrap_yaw_roll(x_heading_up[2])

        # print("Roll: ", self.state[0])
        # print("Pitch: ", self.state[1])
        # print("Yaw: ", self.state[2])
        # print(self.P)