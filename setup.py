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


        self.t_l = []
        self.X_l = []
        self.X_d_l = []
        self.dX_l = []
        self.dX_d_l = []
        self.dX_hat_l = []
        self.ddX_l = []
        self.ddX_hat_l = []
        self.dtheta_l = []
        self.ddtheta_l = []
        self.u_l = []
        self.r_l = []
        self.F_l = []
        self.e_l = []
        self.theta_l = []
    
        self.L = 210
        self.r = 0.65
        self.estimator = Estimator(self.L, self.r)
        self.mhe = EstimatorMHE(self.L, self.r, 100)

        self.estimation_dt = 1 / 200
        self.control_dt = 1 / 300
        self.control_p = Process(target=self.control_process)

        self.alpha = 0.5

        self.u = Value("d", 0)

        self.theta = Value("d", 0)
        self.dtheta = Value("d", 0)
        self.ddtheta = Value("d", 0)
        
        self.X = Value("d", 0)
        self.dX = Value("d", 0)
        self.ddX = Value("d", 0)

        self.F = Value("d", 0)

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

    def init(self, n=10):
        theta_d = n * self.turn
        self.actuator.set_multi_turns_angle(theta_d, self.speed_limit)
        time.sleep(5)
        self.actuator.pause()

    def to_zero(self):
        self.u.value = 0
        if self.control_p.is_alive():
            self.control_p.terminate()
            self.control_p.join()

        self.actuator.set_multi_turns_angle(0, self.speed_limit)
        time.sleep(10)
        self.actuator.turn_off()

    def control_process(self):
        theta = self.actuator.get_multi_turns_angle()
        _theta_p = theta - self.turn * self.turn_counter
        self.turn_counter = theta // self.turn

        self.theta.value = theta
        # dtheta_p = 0
        # ddtheta_p = 0

        x = 0.0
        y = 0.0
        K = 1.0
        T = 0.01

        X_p = self.X.value

        t = time.perf_counter()
        t_p = t
        while True:
            t = time.perf_counter()
            dt = t - t_p

            if dt >= self.control_dt:
                t_p = t

                u = self.u.value
                _, _, dtheta, _theta = self.actuator.set_speed(u) # self.actuator.get_state()
                
                theta = self.multi_turns_encoder(_theta, _theta_p)
                _theta_p = _theta


                # differentiate dtheta
                y = (K / T) * (dtheta - x)         # n
                x = (1 - dt/T) * x + (dt/T)*dtheta # n + 1
                ddtheta = y

                # ddtheta = (dtheta - dtheta_p) / dt
                # dtheta = self.alpha * dtheta + (self.alpha - 1) * dtheta_p
                # dtheta_p = dtheta

                # ddtheta = self.alpha * ddtheta + (self.alpha - 1) * ddtheta_p
                # ddtheta_p = ddtheta

                self.theta.value = theta
                self.dtheta.value = dtheta
                self.ddtheta.value = ddtheta

                X, F, _ddX = self.sensors.get_state()
                dX = (X - X_p) / dt
                X_p = X
                ddX = 1e3 * _ddX # convert to (mm / s**2)

                self.X.value = X
                self.dX.value = dX
                self.ddX.value = ddX

                self.F.value = F


    def start(self, omega_0=3.0, omega_f=3.0, t_f=10.0, X_max=40):
        t_0 = 0.0      # seconds
        X, _, _ = self.sensors.get_state()
        chirp = chirp_trajectory(t_0, t_f, omega_0, omega_f, X, X_max)

        self.X.value = X

        Kp = 10 # control gain

        self.u.value = 0
        self.control_p.start()
        time.sleep(1)

        t_offset = time.perf_counter()
        t = time.perf_counter() - t_offset
        t_p = t

        while t < t_f:

            t = time.perf_counter() - t_offset
            dt = t - t_p

            if dt >= self.estimation_dt:
                t_p = t

                theta = self.theta.value
                dtheta = self.dtheta.value
                ddtheta = self.ddtheta.value

                X = self.X.value
                dX = self.dX.value
                ddX = self.ddX.value

                F = self.F.value

                X_d, dX_d = chirp.get(t)

                inv_J = (self.L - X) / (theta * self.r**2)
                tilde_X = X_d - X
                u = inv_J * (Kp * tilde_X + dX_d)
                self.u.value = u

                dX_hat = (1 / inv_J) * dtheta

                # ddX_hat = self.mhe.get_ddX(theta, dtheta, ddtheta, self.r**2, self.L**2)
                # self.r, self.L, e = self.mhe.estimate(theta, dtheta, ddtheta, ddX)

                ddX_hat = self.mhe.get_ddX(theta, dtheta, ddtheta, self.L, self.r**2)                
                # self.r, e = self.mhe.estimate(theta, dtheta, ddtheta, ddX)

                self.r = self.estimator.update(theta, X)
                e = 0

                self.t_l.append(t)
                self.X_l.append(X)
                self.X_d_l.append(X_d)
                self.dX_l.append(dX)
                self.dX_d_l.append(dX_d)
                self.dX_hat_l.append(dX_hat)
                self.ddX_l.append(ddX)
                self.ddX_hat_l.append(ddX_hat)
                self.theta_l.append(theta)
                self.dtheta_l.append(dtheta)
                self.ddtheta_l.append(ddtheta)
                self.u_l.append(u)
                self.r_l.append(self.r)
                self.F_l.append(F)
                self.e_l.append(e)
        
        self.to_zero()

        return self.t_l, self.X_l, self.X_d_l, self.dX_l, self.dX_d_l, self.dX_hat_l, \
               self.ddX_l, self.ddX_hat_l, self.theta_l, self.dtheta_l,  self.ddtheta_l, \
               self.u_l, self.r_l, self.F_l, self.e_l