# def multiturn_encoder(self):
#     if self.pos_counts_prev - self.pos_counts >= self.turn_threshold:
#         self.motor_turns += 1
#     elif self.pos_counts_prev - self.pos_counts <= -self.turn_threshold:
#         self.motor_turns -= 1
#     self.pos_counts_prev = self.pos_counts

#     return self.pos_counts + (self.encoder_resolution) * self.motor_turns

# counts = self.multiturn_encoder()

import time
import math

# from can import CANSocket
from gyems import GyemsRMD
from sensors import CANSensors
from estimator import EstimatorMHE, EstimatorMHE_both, Estimator

class Setup:
    """ 
        TODO: add description
    """

    def __init__(self, actuator: GyemsRMD, sensors: CANSensors):
        
        self.actuator = actuator
        self.actuator.turn_off()

        # accel_limit = 15000
        # self.actuator.set_accel_limit(accel_limit)

        # gains = (40, 40, 40, 40, 40, 40)
        # self.actuator.set_gains(gains, "RAM")

        self.sensors = sensors
        self.sensors.reset()
        # self.sensors.calibrate_force()

        self.turn = 2 * math.pi
        self.speed_limit = 16 * self.turn

        self.L = 210
        self.r = 0.9
        # self.r = 0.84
        self.estimator = Estimator(self.L, self.r)
        self.mhe = EstimatorMHE_both(self.L, self.r, 100)

    def start(self):
        theta_d = 10 * self.turn
        self.actuator.set_multi_turns_angle(theta_d, self.speed_limit)
        time.sleep(5)
        self.actuator.pause()

    def control(self, Kp=10, Kd=4):
        X_0, _, _ = self.sensors.get_state()
        X_max = 40
        omega_0 = 3.0
        omega_f = 3.0
        
        t_l = []
        X_l = []
        X_d_l = []
        dX_d_l = []
        dX_hat_l = []
        ddX_l = []
        ddX_hat_l = []
        dtheta_l = []
        ddtheta_l = []
        u_l = []
        r_l = []
        F_l = []
        e_l = []

        dtheta_p = 0

        t_offset = time.perf_counter()
        t_0 = time.perf_counter() - t_offset
        t_f = 30.0 # seconds

        chirp = chirp_trajectory(t_0, t_f, omega_0, omega_f, X_0, X_max)

        t = t_0
        t_p = t_0

        t_log = 1 / 200

        while t < t_f:
            X_d, dX_d = chirp.get(t)

            t = time.perf_counter() - t_offset
            dt = t - t_p

            if dt >= t_log:  
                t_p = t

                theta = self.actuator.get_multi_turns_angle()
                _, _, dtheta, _ = self.actuator.get_state()
                ddtheta = (dtheta - dtheta_p) / dt
                dtheta_p = dtheta

                X, F, ddX = self.sensors.get_state()
                ddX = 1e3 * ddX # convert to (mm / s**2)

                self.r = self.estimator.update(theta, X)
                # self.r, self.L, e = self.mhe.estimate(theta, dtheta, ddtheta, ddX)
                e = 0

                inv_J = (self.L - X) / (theta * self.r**2)

                tilde_X = X_d - X
                u = inv_J * (Kp * tilde_X + dX_d)
                self.actuator.set_speed(u)

                dX_hat = (1 / inv_J) * dtheta
                ddX_hat = self.mhe.get_ddX(theta, dtheta, ddtheta, self.r**2, self.L**2)

                t_l.append(t)
                X_l.append(X)
                X_d_l.append(X_d)
                dX_d_l.append(dX_d)
                dX_hat_l.append(dX_hat)
                ddX_l.append(ddX)
                ddX_hat_l.append(ddX_hat)
                dtheta_l.append(dtheta)
                ddtheta_l.append(ddtheta)
                u_l.append(u)
                r_l.append(self.r)
                F_l.append(F)
                e_l.append(e)

        return t_l, X_l, X_d_l, dX_d_l, dX_hat_l, ddX_l, ddX_hat_l, \
            dtheta_l, ddtheta_l, u_l, r_l, F_l, e_l

    def stop(self):
        self.actuator.set_multi_turns_angle(0, self.speed_limit)
        time.sleep(10)
        self.actuator.turn_off()

class chirp_trajectory:
    """ 
        TODO: add description
    """

    def __init__(self, t_0, t_f, omega_0, omega_f, x_0, x_max):
        
        self.t_0 = t_0
        self.t_f = t_f

        self.omega_0 = omega_0
        self.omega_f = omega_f
        self.domega = (omega_f - omega_0)/(t_f - t_0)

        self.x_0 = x_0
        self.x_max = x_max

    def get(self, t):
        omega = self.omega_0 + self.domega * t
        
        x = (1/2) * self.x_max * (1 - math.cos(omega * t)) + self.x_0
        dx = (1/2) * self.x_max * (omega + t * self.domega) \
            * math.sin(omega * t)

        return x, dx