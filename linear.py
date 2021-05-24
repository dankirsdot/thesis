import math
from time import sleep
from multiprocessing import Process, Value


class LinearSetup:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    def __init__(self, sensors=None, actuator=None):
        self.sensors = sensors
        
        self.actuator = actuator
        self.actuator["control"] = Value("d", 0)

        self.state = {}
        self.sensors_data = {}
        self.state_labels = {"theta", "dtheta",
                             "x", "dx", "current", "force", "accel"}
        for l in self.state_labels:
            self.state[l] = 0
            self.sensors_data[l] = Value("d", 0)

        self.pool = []

        self.to_home = True

        self.multiturn_angle = Value("d", 0)

    def run(self):
        """Run the sensing and motor processes"""
        self.pool.append(
            Process(target=self.sensing_process)
        )
        self.pool.append(
            Process(target=self.motor_process)
        )
        # self.pool.append(
            # Process(target=self.multiturn_encoder_process)
        # )
        print("Processes are about to start...")
        for p in self.pool:
            p.start()

    def disable(self):
        self.actuator["motor"].disable()

    def enable(self):
        self.actuator["motor"].enable()

    def stop(self, delay=0.0):
        sleep(delay)

        print("Processes are about to stop...")
        for p in self.pool:
            p.terminate()

        if self.to_home:
            print(self.sensors_data['theta'].value)
            self.actuator["motor"].set_angle(0, speed_limit=1000)
            self.sensors_data['theta'].value = self.actuator['motor'].state['angle'] - \
                self.actuator['angle_offset']
            while abs(self.sensors_data['theta'].value) > 0.01:
                self.actuator["motor"].set_angle(0, speed_limit=1000)
                self.sensors_data['theta'].value = self.actuator['motor'].state['angle'] - \
                    self.actuator['angle_offset']
            print(self.sensors_data['theta'].value)

        self.disable()
        print("Processes are terminated...")

    def sensing_process(self):
        print("Sensing procces is launched")
        while True:
            self.sensors.request_reply()
            self.sensors.parse_data()
            self.sensors_data["x"].value = self.sensors.data['lin'][0]
            self.sensors_data["dx"].value = self.sensors.differences['lin']
            self.sensors_data["force"].value = self.sensors.data['frc'][0]
            self.sensors_data["accel"].value = self.sensors.data['acc'][0]

    def motor_process(self):
        print("Motor procces is launched")
        # for actuator in self.actuators_labels:
        self.actuator["motor"].enable()

        while True:
            u = self.actuator["control"].value
            # self.actuator["motor"].set_current(u)
            self.actuator["motor"].set_speed(u)

            self.sensors_data["theta"].value = self.actuator["motor"].state["angle"] - \
                self.actuator["angle_offset"]
            self.sensors_data["dtheta"].value = self.actuator["motor"].state["speed"]
            self.sensors_data["current"].value = self.actuator["motor"].state["torque"]

    def multiturn_encoder_process(self):
        self.multiturn_angle.value =  2*math.pi*self.actuator["motor"].get_multiturn_angle()/(2**64-1)
        return self.multiturn_angle.value

    def get_state(self):
        for state in self.state_labels:
            self.state[state] = self.sensors_data[state].value
        return self.state

    def set_control(self, control):
        """Update the value for controller with arguments"""
        # TODO: think on what is appropriate argument
        # for actuator in self.actuators_labels:
        self.actuator["control"].value = control
