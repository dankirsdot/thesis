import time
import math

from multiprocessing import Process, Value

# from can import CANSocket
from gyems import GyemsRMD
from sensors import CANSensors
from trajectory import chirp_trajectory
from estimator import EstimatorMHE, EstimatorMHE_both, Estimator

class Setup:
    """ 
        TODO: add description
    """

    def __init__(self, actuator: GyemsRMD, sensors: CANSensors):
        
        self.actuator = actuator
        self.actuator.turn_off()

        self.sensors = sensors
        self.sensors.reset()
        # self.sensors.calibrate_force()

        self.L = 210
        self.r = 0.9
        self.estimator = Estimator(self.L, self.r)
        self.mhe = EstimatorMHE(self.L, self.r, 100)

        self.process_pool = []

        self.turn = 2 * math.pi
        self.speed_limit = 16 * self.turn

        self.turn_counter = 0
        self.turn_threshold = math.pi

    def multi_turns_encoder(self, _theta, _theta_p):
        if _theta_p - _theta >= self.turn_threshold:
            self.turn_counter += 1
        elif _theta_p - _theta <= -self.turn_threshold:
            self.turn_counter += -1
        return _theta + self.turn * self.turn_counter

    def start(self):
        theta_d = 10 * self.turn
        self.actuator.set_multi_turns_angle(theta_d, self.speed_limit)
        time.sleep(5)
        self.actuator.pause()

    def control(self, Kp=10, Kd=4):        
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
        theta_l = []

        dtheta_p = 0

        X, _, _ = self.sensors.get_state()
        
        theta = self.actuator.get_multi_turns_angle()
        self.turn_counter = theta // self.turn
        _theta_p = theta - self.turn * self.turn_counter

        t_log = 1 / 400

        t_0 = 0.0     # seconds
        t_f = 30.0    
        X_max = 40    # amplitude
        omega_0 = 3.0 # initial frequency (rad)
        omega_f = 3.0 # final frequency (rad)
        chirp = chirp_trajectory(t_0, t_f, omega_0, omega_f, X, X_max)

        t_offset = time.perf_counter()
        t_0 = time.perf_counter() - t_offset

        t = t_0
        t_p = t_0

        while t < t_f:

            t = time.perf_counter() - t_offset
            dt = t - t_p

            if dt >= t_log:
                t_p = t

                X_d, dX_d = chirp.get(t)

                inv_J = (self.L - X) / (theta * self.r**2)
            
                tilde_X = X_d - X
            
                u = inv_J * (Kp * tilde_X + dX_d)
                
                _, _, dtheta, _theta = self.actuator.set_speed(u)
                X, F, _ddX = self.sensors.get_state()

                theta = self.multi_turns_encoder(_theta, _theta_p)
                _theta_p = _theta

                ddtheta = (dtheta - dtheta_p) / dt
                dtheta_p = dtheta

                dX_hat = (1 / inv_J) * dtheta

                ddX = 1e3 * _ddX # convert to (mm / s**2)
                # ddX_hat = self.mhe.get_ddX(theta, dtheta, ddtheta, self.r**2, self.L**2)
                # self.r, self.L, e = self.mhe.estimate(theta, dtheta, ddtheta, ddX)

                ddX_hat = self.mhe.get_ddX(theta, dtheta, ddtheta, self.L, self.r**2)                
                self.r, e = self.mhe.estimate(theta, dtheta, ddtheta, ddX)
                print(self.r)

                # self.r = self.estimator.update(theta, X)
                # e = 0

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
                theta_l.append(theta)

        return t_l, X_l, X_d_l, dX_d_l, dX_hat_l, ddX_l, ddX_hat_l, \
            dtheta_l, ddtheta_l, u_l, r_l, F_l, e_l, theta_l

    def stop(self):
        self.actuator.set_multi_turns_angle(0, self.speed_limit)
        time.sleep(10)
        self.actuator.turn_off()