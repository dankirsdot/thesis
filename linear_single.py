import math
import numpy as np
from time import sleep
from multiprocessing import Process, Value


class LinearSetup:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    def __init__(self, sensors=None, actuator=None):
        self.sensors = sensors
        
        self.actuator = actuator
        self.actuator["control"] = Value("d", 0)

        self.state = {}
        self.offset = {}
        self.sensors_data = {}
        self.state_labels = {"theta", "dtheta",
                             "x", "dx", "current", "force", "accel"}
        for l in self.state_labels:
            self.offset[l] = 0
            self.state[l] = 0
            self.sensors_data[l] = Value("d", 0)

        self.pool = []

        self.to_home = True

        self.multiturn_angle = Value("d", 0)

    def run(self):
        """Run the sensing and motor processes"""
        self.pool.append(
            Process(target=self.control_process)
        )
        print("Processes are about to start...")
        for p in self.pool:
            p.start()

    def disable(self):
        self.actuator["motor"].disable()

    def pause(self):
        self.actuator["motor"].pause()

    def enable(self):
        self.actuator["motor"].enable()

    def stop(self, delay=0.0):
        sleep(delay)

        print("Processes are about to stop...")
        for p in self.pool:
            p.terminate()

        if self.to_home:
            theta = self.actuator["motor"].get_multiturn_angle(radians=False) - self.offset["theta"]
            while not math.isclose(0, theta, abs_tol=10):
                self.actuator["motor"].set_angle(0, speed_limit=500)
                theta = self.actuator["motor"].get_multiturn_angle(radians=False) - self.offset["theta"]
            print("Current position:", theta)

        self.disable()
        print("Processes are terminated...")

    def control_process(self):
        self.actuator["motor"].enable()
        print("Control procces is launched")
        while True:
            u = self.actuator["control"].value
            # self.actuator["motor"].set_current(u)
            self.actuator["motor"].set_speed(u)

            # actuator
            self.sensors_data["theta"].value = self.actuator["motor"].state["angle"] - \
                self.offset["theta"]
            self.sensors_data["dtheta"].value = self.actuator["motor"].state["speed"]
            self.sensors_data["current"].value = self.actuator["motor"].state["torque"]

            # sensors
            self.sensors.request_reply()
            self.sensors.parse_data()
            self.sensors_data["x"].value = self.sensors.data['lin'][0] - self.offset["x"]
            self.sensors_data["dx"].value = self.sensors.differences['lin']
            self.sensors_data["force"].value = self.sensors.data['frc'][0]
            self.sensors_data["accel"].value = self.sensors.data['acc'][0]

            # print(self.actuator["motor"].get_multiturn_angle())

    def get_state(self):
        for state in self.state_labels:
            self.state[state] = self.sensors_data[state].value
        return self.state

    def set_control(self, control):
        """Update the value for controller with arguments"""
        # TODO: think on what is appropriate argument
        # for actuator in self.actuators_labels:
        self.actuator["control"].value = control

    def go_to_angle(self, theta_d, radians=True):
        if radians:
            theta_d = theta_d * (180 / math.pi)
        theta_d_counts = 100 * theta_d

        theta = self.actuator["motor"].get_multiturn_angle(radians=False)
        while not math.isclose(theta_d, theta, abs_tol=10):
            self.actuator["motor"].set_angle(theta_d_counts, speed_limit=500)
            theta = self.actuator["motor"].get_multiturn_angle(radians=False)

    def find_zero(self, turns=5):
        list_theta = []
        list_X = []

        one_turn_counts = 36000 # represents 360 degrees
        theta_d_counts = turns * one_turn_counts # integer we send to the motor
        theta_d = 0.01 * theta_d_counts # 0.01 degree corresponds to 1 bit
        theta = self.actuator["motor"].get_multiturn_angle(radians=False)
        while not math.isclose(theta_d, theta, abs_tol=50):
            self.actuator["motor"].set_angle(theta_d_counts, speed_limit=100)
            theta = self.actuator["motor"].get_multiturn_angle(radians=False)
            list_theta.append(theta)

            self.sensors.request_reply()
            self.sensors.parse_data()
            X = self.sensors.data['lin'][0]
            list_X.append(X)

        while not math.isclose(-theta_d, theta, abs_tol=50):
            self.actuator["motor"].set_angle(-theta_d_counts, speed_limit=100)
            theta = self.actuator["motor"].get_multiturn_angle(radians=False)
            list_theta.append(theta)

            self.sensors.request_reply()
            self.sensors.parse_data()
            X = self.sensors.data['lin'][0]
            list_X.append(X)
    
        np_theta = np.array(list_theta)
        np_X = np.array(list_X)
        argmin = np.argmin(np_X)
        X_min = np_X[argmin]
 
        theta_min = np_theta[argmin]
        theta_min_counts = int(100 * theta_min)

        self.offset["theta"] = theta_min * (math.pi / 180) # in rads
        self.offset["x"] = X_min # in mm

        while not math.isclose(theta_min, theta, abs_tol=10):
            self.actuator["motor"].set_angle(theta_min_counts, speed_limit=100)
            theta = self.actuator["motor"].get_multiturn_angle(radians=False)

            self.sensors.request_reply()
            self.sensors.parse_data()
            X = self.sensors.data['lin'][0]

        print(theta_min, theta)

    def calibrate_force(self, mass, n=10000):
        '''
        n - number of measurments
        mass - mass used for calibration
        '''

        g = 9.81
        actual_weight = mass * g

        print("Force sensor calibration started")
        input("Press enter in order to measure bias")
        force_offset = 0
        for i in range(n):
            self.sensors.request_reply()
            self.sensors.parse_data()
            force_offset += self.sensors.counts['frc'][0]
        force_offset /= n

        input("Place mass and press enter in order to extimate scale")
        forces_scale = 0
        for i in range(n):
            self.sensors.request_reply()
            self.sensors.parse_data()
            count = self.sensors.counts['frc'][0]
            forces_scale += actual_weight / (count - force_offset)
        forces_scale /= n

        self.sensors.scales['frc'] = forces_scale
        self.sensors.offsets['frc'] = force_offset
        print("New force scale:", self.sensors.scales['frc'])
        print("New force offset:", self.sensors.offsets['frc'])
        print("Force sensor calibration finished")
     