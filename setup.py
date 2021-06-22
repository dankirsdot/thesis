import time
import math

from multiprocessing import Process, Value

# from can import CANSocket
from gyems import GyemsRMD
from sensors import CANSensors
from trajectory import chirp_trajectory
from estimator import EstimatorMHE, EstimatorMHE_new, Estimator

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
        self.X_hat_l = []
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

        self.r_mhe_l = []
        self.e_mhe_l = []

        self.L = 210
        self.r = 0.7
        self.estimator = Estimator(self.L, self.r)
        self.mhe = EstimatorMHE(self.L, self.r, 10)

        self.estimation_dt = 1 / 200
        self.control_dt = 1 / 200
        self.control_p = Process(target=self.control_process)

        self.u = Value("d", 0)

        self.theta = Value("d", 0)
        self.dtheta = Value("d", 0)
        self.ddtheta = Value("d", 0)

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
        self.turn_counter = theta // self.turn
        _theta_p = theta - self.turn * self.turn_counter

        self.theta.value = theta

        # discrete filtered derivative
        # https://www.mathworks.com/help/physmod/sps/ref/filteredderivativediscreteorcontinuous.html
        x = 0.0
        y = 0.0
        K = 1.0
        T = 0.05
        
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

                # discrete filtered derivative
                y = (K / T) * (dtheta - x)         # n
                x = (1 - dt/T) * x + (dt/T)*dtheta # n + 1
                ddtheta = y

                self.theta.value = theta
                self.dtheta.value = dtheta
                self.ddtheta.value = ddtheta

                # print(1/dt)

    def get_ddX_hat(self, theta, dtheta, ddtheta, r, L):
        ddX_hat = (r**2 / math.sqrt(L**2 - theta**2 * r**2)) * \
            (theta * ddtheta + dtheta**2 * ((theta**2 * r**2)/(L**2 - theta**2 * r**2) + 1))
        return ddX_hat

    def start(self, omega_0=3.0, omega_f=3.0, t_f=10.0, X_max=40):
        t_0 = 0.0      # seconds
        X, _, _ddX = self.sensors.get_state()
        chirp = chirp_trajectory(t_0, t_f, omega_0, omega_f, X, X_max)

        Kp = 10 # control gain

        X_p = X

        # low-pass filter
        # https://www.mathworks.com/help/physmod/sps/ref/lowpassfilterdiscreteorcontinuous.html
        x = 0
        y = 0
        K = 1.0
        T = 0.005

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

                X, F, _ddX = self.sensors.get_state()

                theta = self.theta.value
                dtheta = self.dtheta.value
                ddtheta = self.ddtheta.value
                
                # low-pass filter
                y = x                              # n
                x = (1 - dt/T) * x + K*(dt/T)*_ddX # n + 1
                _ddX = y

                ddX = 1e3 * _ddX # convert to (mm / s**2)

                dX = (X - X_p) / dt
                X_p = X

                X_d, dX_d = chirp.get(t)

                inv_J = (self.L - X) / (theta * self.r**2)
                tilde_X = X_d - X
                u = inv_J * (Kp * tilde_X + dX_d)
                self.u.value = u

                X_hat = self.L - math.sqrt(self.L**2 - theta**2 * self.r**2)
                dX_hat = (1 / inv_J) * dtheta

                ddX_hat = self.mhe.get_ddX(theta, dtheta, ddtheta, self.L, self.r**2)                
                r_mhe, e_mhe = self.mhe.estimate(theta, dtheta, ddtheta, ddX)
                #r_mhe = 0
                #e_mhe = 0

                r = self.estimator.update(theta, X)
                # ddX_hat = self.get_ddX_hat(theta, dtheta, ddtheta, self.r, self.L)
                e = ddX_hat - ddX

                self.r = r_mhe

                self.t_l.append(t)
                self.X_l.append(X)
                self.X_d_l.append(X_d)
                self.X_hat_l.append(X_hat)
                self.dX_l.append(dX)
                self.dX_d_l.append(dX_d)
                self.dX_hat_l.append(dX_hat)
                self.ddX_l.append(ddX)
                self.ddX_hat_l.append(ddX_hat)
                self.theta_l.append(theta)
                self.dtheta_l.append(dtheta)
                self.ddtheta_l.append(ddtheta)
                self.u_l.append(u)
                self.r_l.append(r)
                self.F_l.append(F)
                self.e_l.append(e)

                self.r_mhe_l.append(r_mhe)
                self.e_mhe_l.append(e_mhe)
        
        self.to_zero()

        return self.t_l, self.X_l, self.X_d_l, self.X_hat_l, self.dX_l, self.dX_d_l, self.dX_hat_l, \
               self.ddX_l, self.ddX_hat_l, self.theta_l, self.dtheta_l,  self.ddtheta_l, \
               self.u_l, self.r_l, self.F_l, self.e_l, self.r_mhe_l, self.e_mhe_l
