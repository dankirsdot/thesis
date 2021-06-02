import time
import math

import numpy as np
import matplotlib.pyplot as plt

from can import CANSocket
from sensors import CANSensors

_t = []
_az = []
_az_num = []
g = 9.8067

if __name__ == '__main__':
    try:
        sock = CANSocket(interface='can1')
        sensors = CANSensors(sock, id=0x01)

        t_f = 5 # final time

        t_p = 0 # previous time
        t_log = 5e-2

        t_0 = time.perf_counter()
        vel_p = 0
        pos_p, force_p, accel_p = sensors.get_state()
        pos_p = pos_p / 1e3

        sensors.get_imu_settings()

        while True:
            t = time.perf_counter() - t_0
            dt = t - t_p

            pos, force, accel = sensors.get_state()
            # print(force)

            # pos = pos / 1e3
            # vel = (pos - pos_p) / dt
            # az_num = (vel - vel_p) / dt
            # az_num = np.clip(az_num, -10, 10)

            # pos_p = pos
            # vel_p = vel

            # az = accel + 1

            if dt >= t_log:
                print(force)
            #     # log some data
            #     _t.append(t)
            #     _az.append(az * g)
            #     _az_num.append(az_num)
            #     t_p = t

            if t >= t_f:
                raise KeyboardInterrupt

    except Exception as e:
        print(e)
    finally:
        plt.plot(_t, _az_num)
        plt.plot(_t, _az)
        plt.savefig("az.png")