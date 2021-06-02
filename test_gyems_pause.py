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

        turn = 2*math.pi
        angle = 20 * turn
        speed = 2 * turn
        actuator.set_multi_turns_angle(angle, speed)
        time.sleep(5)

        actuator.pause()
        time.sleep(1)
        actuator.resume()
        time.sleep(10)

        actuator.set_multi_turns_angle(0, speed)
        time.sleep(10)

    except Exception as e:
        print(e)
    finally:
        actuator.set_multi_turns_angle(0, speed_limit=10)
        time.sleep(5)
        actuator.turn_off()