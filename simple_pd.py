import time
import math
import numpy as np
from can import CANSocket
from sensors import CANSensors
from gyems import GyemsDRC
from multiprocessing import Process, Value
# from linear import LinearSetup
from linear_single import LinearSetup
import matplotlib.pyplot as plt

if __name__ == '__main__':
    try:
        can_bus = CANSocket(interface='can1')

        # Initialize sensors
        sensors_id = 0x01
        sensors_labels = ['lin', 'frc', 'acc']
        sensors = CANSensors(can_bus, sensors_id, sensors_labels)

        sensors.data_map = {'lin': [0, 2], 'frc': [2, 4], 'acc': [4, 6]}
        sensors.scales = {'lin': 25.4 /
                          (360 * 4), 'frc': 1.0, 'acc': 0.000061}
        sensors.offsets = {'lin': 0, 'frc': 0, 'acc': 0.602295}
        sensors.reset_counters(sensors=['lin'], output=True)
        sensors.set_differences({'lin'})

        # Initialize actuator
        motor_id = 0x141
        actuator = {"motor": GyemsDRC(can_bus, motor_id),
                    "limit": 2000,
                    "angle_offset": 0,
                    "pos_offset": 0,
                    }

        actuator["motor"].reset()
        actuator["motor"].set_radians()
        actuator["motor"].current_limit = actuator["limit"]
        actuator["torque_constant"] = 1

        # gains = [40, 40, 40, 40, 40, 40]
        # actuator["motor"].set_pid(gains)

        setup = LinearSetup(sensors, actuator)
        setup.to_home = True

        t_prev = 0
        t0 = time.perf_counter()

        A0 = 0
        A = 50
        nu = 2 * math.pi * 0.5
        kp, kd = 10, 4

        t_log = 5e-3
        t_final = 10
        
        data = {}
        labels = ['time', 'x', 'dx', 'theta', 'dtheta', 'accel', 'force', 'torque', 'u', 'theta_d', 'dtheta_d']
        for l in labels:
            data[l] = []

        setup.run()
        time.sleep(1.0)

        while True:
            t = time.perf_counter() - t0
            dt = t - t_prev
            
            state = setup.get_state()
            theta, dtheta = state['theta'], state['dtheta']
            x, dx = state['x'], state['dx']
            force, torque = state['force'], state['current']*0.212
            accel = state['accel']
            
            theta_d = A0 + A * math.sin(nu * t)
            dtheta_d = A * nu * math.cos(nu * t)

            e = theta_d - theta
            de = dtheta_d - dtheta

            u = kp*e + dtheta_d

            setup.set_control(u)

            if dt >= t_log:
                data['time'].append(t)
                data['x'].append(x)
                data['dx'].append(dx)
                data['theta'].append(theta)
                data['dtheta'].append(dtheta)
                data['accel'].append(accel)
                data['force'].append(force)
                data['torque'].append(torque)
                data['u'].append(u)
                data['theta_d'].append(theta_d)
                data['dtheta_d'].append(dtheta_d)
                t_prev = t

            if t >= t_final:
                raise KeyboardInterrupt
            # print(f'\r {round(x, 3)}, {round(dx, 3)}, {round(theta_d, 3)}, {round(theta, 3)}, {round(dtheta, 3)}, {round(accel, 3)}, {round(force, 3)}, {round(torque , 3)}', end=8*" ", flush=True)

    except KeyboardInterrupt:
        print("KeyboardInterrupt!!!")
    finally:
        setup.stop()

        t = np.array(data['time'])[:] - np.array(data['time'])[0]
        x =  data['x']
        dx =  data['dx']
        theta =  data['theta']
        dtheta = data['dtheta']
        theta_d = data['theta_d']
        dtheta_d = data['dtheta_d'] 
        u = data['u']
        accel = data['accel']
        e = np.array(theta_d) - np.array(theta)

        plt.figure(figsize=(7,3))
        plt.title(r'Motor Angle $\theta(t)$')
        plt.plot(t, theta, 'r', linewidth=1.5, label = r'$\theta$', zorder = 2)
        plt.plot(t, theta_d, 'g', linewidth=1.5, label = r'$\theta_d$', zorder = 2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Angle $\theta(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/angle.png', dpi=300)

        plt.figure(figsize=(7,3))
        plt.title(r'Motor Angle Error $e(t)$')
        plt.plot(t, e, 'r', linewidth=1.5, label = r'$e$', zorder = 2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.xlim([t[1000], t[-1]])
        plt.ylim([np.max(e[1000:]), np.min(e[1000:])])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Angle $e(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/error.png', dpi=300)

        plt.figure(figsize=(7,3))
        plt.title(r'Contraction $X(t)$')
        plt.plot(t, x, 'b', linewidth=1.5, label = r'X', zorder = 2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Contraction $X(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/contraction.png', dpi=300)

        plt.figure(figsize=(7,3))
        plt.title(r'Contraction Speed $\dot{X}(t)$')
        plt.plot(t, x, 'b', linewidth=1.5, label = r'$\dot{X}$', zorder = 2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Contraction speed $\dot{X}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/contraction_speed.png', dpi=300)

        plt.figure(figsize=(4,6))
        plt.title(r'Phase Plane $\dot{\theta}(\theta)$')
        plt.plot(theta, dtheta, 'r', linewidth=1.5, label = r'plane', zorder = 2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.xlabel(r'Angle $\theta(t)$')
        plt.ylabel(r'Speed $\dot{\theta}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/phpln_theta.png', dpi=300)

        plt.figure(figsize=(7,3))
        plt.title(r'Motor Speed $\dot{\theta}(t)$')
        plt.plot(t, dtheta, 'r', linewidth=1.5, label = r'$\dot{\theta}$', zorder = 2)
        plt.plot(t, dtheta_d, 'g', linewidth=1.5, label = r'$\dot{\theta}_d$', zorder = 2)
        plt.plot(t, u, 'b', linewidth=1.5, label = r'$u$', zorder = 2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Speed $\dot{\theta}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/speed.png', dpi=300)

        plt.figure(figsize=(7,3))
        plt.title(r'Linear acceleration $\ddot{X}(t)$')
        plt.plot(t, accel, 'r', linewidth=1.5, label = r'$\ddot{X}$', zorder = 2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Speed $\ddot{X}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/accel.png', dpi=300)