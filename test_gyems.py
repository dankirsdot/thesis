import time
import math

import numpy as np
import matplotlib.pyplot as plt

from can import CANSocket
from gyems import GyemsRMD


if __name__ == '__main__':
    try:
        sock = CANSocket(interface='can1')
        actuator = GyemsRMD(sock, id=0x141)

        # TODO: check correctness of the motor speed measurements
        # TODO: check correctness of the phase current measurements
        # TODO: checl turn_off, pause, resume functions

        # while True:
        #     actuator.set_multi_turns_angle(multi_turns_counts, speed_limit)
        #     time.sleep(2)
        #     actuator.get_multi_turns_angle()
        #     print(actuator.multi_turns_counts - multi_turns_counts)

        #     actuator.set_multi_turns_angle(0.*multi_turns_counts, speed_limit)
        #     time.sleep(2)
        #     actuator.get_multi_turns_angle()
        #     print(actuator.multi_turns_counts)

        t_f = 10 # final time

        t_p = 0 # previous time
        t_log = 5e-3
        
        A0 = 0
        A = 20
        nu = 2 * math.pi * 0.5
        kp, kd = 4, 4

        t0 = time.perf_counter()

        _, current, dtheta, _ = actuator.get_state()
        theta = actuator.get_multi_turns_angle()

        while True:
            t = time.perf_counter() - t0
            dt = t - t_p

            theta_d = A0 + A * math.sin(nu * t)
            dtheta_d = A * nu * math.cos(nu * t)

            e = theta_d - theta
            de = dtheta_d - dtheta

            u = kp*e + kd*dtheta_d

            _, current, dtheta, _ = actuator.set_speed(u)
            theta = actuator.get_multi_turns_angle()

            if dt >= t_log:
                # log some data 
                t_p = t

            if t >= t_f:
                raise KeyboardInterrupt

    except Exception as e:
        print(e)
    finally:
        actuator.set_multi_turns_angle(0, speed_limit=10)
        time.sleep(5)
        actuator.turn_off()