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
                          (360 * 4), 'frc': 0.166, 'acc': 0.000061}
        sensors.offsets = {'lin': 0, 'frc': 1764, 'acc': 0.602295}
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

        gains = [80, 80, 40, 40, 40, 40]
        actuator["motor"].set_pid(gains)

        setup = LinearSetup(sensors, actuator)
        setup.to_home = True
        # setup.find_zero()

        initial_angle = 50.0 # in radians
        setup.go_to_angle(initial_angle)

        L = 210
        r = 0.63
        psi = 1 / r**2

        A0 = 0
        A = 50
        nu = 2 * math.pi * 0.5

        X_0 = 5
        X_max = 20

        kp, kd = 8, 4

        data = {}
        labels = ['time', 'x', 'dx', 'theta', 'dtheta',
                  'force', 'torque', 'u', 'accel', 'psi',
                  'x_d', 'dx_d', 'theta_d', 'dtheta_d']
        for l in labels:
            data[l] = []

        setup.run()
        time.sleep(0.1)

        omega_0 = 1.0
        omega_f = 3.0
        omega_p = omega_0

        t_log = 5e-3
        t_offset = time.perf_counter()
        t_0 = time.perf_counter() - t_offset
        t_f = 20
        t_p = t_0
        
        k = (omega_f - omega_0)/(t_f - t_0)
 
        N = 0
        sum_HH = 0

        while True:
            t = time.perf_counter() - t_offset
            dt = t - t_p

            state = setup.get_state()
            theta, dtheta = state['theta'], state['dtheta']
            x, dx = state['x'], state['dx']
            force, torque = state['force'], state['current']*0.212
            accel = state['accel']

            theta_d = A0 + A * math.sin(nu * t)
            dtheta_d = A * nu * math.cos(nu * t)

            # e = theta_d - theta
            # de = dtheta_d - dtheta

            # u = kp*e + dtheta_d

            ### test control ###
            omega = omega_0 + k*t
            domega = (omega - omega_p) / dt
            omega_p = omega

            if t > 0 and t <= 2:
                x_d = 30
            elif t > 2 and t <= 4:
                x_d = 20
            elif t > 4 and t <= 6:
                x_d = 40
            elif t > 6 and t <= 8:
                x_d = 30
            elif t > 8 and t <= 10:
                x_d = 50
            else:
                x_d = 5

            # x_d = 30 #(1/2) * X_max * (1 - math.cos(omega * t)) + X_0
            dx_d = 0 #(1/2) * X_max * (omega + t*domega) * math.sin(omega * t)

            # if theta < 1e-3 and theta >= 0.0:
            #     theta = theta + 1e-3
            # elif theta > -1e-3 and theta < 0.0:
            #     theta = theta - 1e-3
            
            # estimate psi
            N += 1
            Gamma = 1e-5
            z = theta**2
            H = 2*x*L - x**2
            sum_HH += H*H
            R = sum_HH / N
            inv_R = 1 / R
            e = z - H*psi
            psi = psi + Gamma * inv_R * H * e

            inv_pX_ptheta = psi * ((L - x) / theta)

            tilde_x = x_d - x
            u =  inv_pX_ptheta * (kp * tilde_x + dx_d)

            # if math.isclose(x, 5, abs_tol=0.01):
            #     print(x, theta)
            ####################


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
                data['x_d'].append(x_d)
                data['dx_d'].append(dx_d)
                data['theta_d'].append(theta_d)
                data['dtheta_d'].append(dtheta_d)
                data['psi'].append(psi)
                t_p = t

            if t >= t_f:
                raise Exception

    except Exception as e:
        print("Error!", e)
    finally:
        setup.stop()

        print("Plotting")
        t = np.array(data['time'])[:] - np.array(data['time'])[0]
        x = data['x']
        dx = data['dx']
        x_d = data['x_d']
        dx_d = data['dx_d']
        theta = data['theta']
        dtheta = data['dtheta']
        theta_d = data['theta_d']
        dtheta_d = data['dtheta_d']
        u = data['u']
        psi = data['psi']
        accel = data['accel']
        
        e_x = np.array(x_d) - np.array(x)
        e_theta = np.array(theta_d) - np.array(theta)

        r = np.sqrt(1 / np.array(psi))

        plt.figure(figsize=(7, 3))
        plt.title(r'Motor Angle $\theta(t)$')
        plt.plot(t, theta, 'r', linewidth=1.5, label=r'$\theta$', zorder=2)
        plt.plot(t, theta_d, 'g', linewidth=1.5, label=r'$\theta_d$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Angle $\theta(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/angle.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Motor Angle Error $e_{\theta}(t)$')
        plt.plot(t, e_theta, 'r', linewidth=1.5, label=r'$e_{\theta}$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([t[1000], t[-1]])
        plt.ylim([np.max(e_theta[1000:]), np.min(e_theta[1000:])])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Angle error $e_{\theta}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/angle_error.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction $X(t)$')
        plt.plot(t, x, 'r', linewidth=1.5, label=r'$X$', zorder=2)
        plt.plot(t, x_d, 'g', linewidth=1.5, label=r'$X_d$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Contraction $X(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/contraction.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction Error $e_X(t)$')
        plt.plot(t, e_x, 'r', linewidth=1.5, label=r'$e_X$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([t[1000], t[-1]])
        plt.ylim([np.max(e_x[1000:]), np.min(e_x[1000:])])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Contraction error $e_X(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/contraction_error.png', dpi=300)
        
        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction Speed $\dot{X}(t)$')
        plt.plot(t, dx, 'r', linewidth=1.5, label=r'$\dot{X}$', zorder=2)
        plt.plot(t, dx_d, 'g', linewidth=1.5, label=r'$\dot{X}_d$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Contraction speed $\dot{X}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/contraction_speed.png', dpi=300)

        plt.figure(figsize=(4, 6))
        plt.title(r'Phase Plane $\dot{\theta}(\theta)$')
        plt.plot(theta, dtheta, 'r', linewidth=1.5, label=r'plane', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlabel(r'Angle $\theta(t)$')
        plt.ylabel(r'Speed $\dot{\theta}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/phpln_theta.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Motor Speed $\dot{\theta}(t)$')
        plt.plot(t, dtheta, 'r', linewidth=1.5,
                 label=r'$\dot{\theta}$', zorder=2)
        plt.plot(t, dtheta_d, 'g', linewidth=1.5,
                 label=r'$\dot{\theta}_d$', zorder=2)
        plt.plot(t, u, 'b', linewidth=1.5, label=r'$u$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Speed $\dot{\theta}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/speed.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Linear acceleration $\ddot{X}(t)$')
        plt.plot(t, accel, 'r', linewidth=1.5, label=r'$\ddot{X}$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Speed $\ddot{X}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/accel.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Radius $r(t)$')
        plt.plot(t, r, 'r', linewidth=1.5, label=r'$r$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Radius $r(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/radius.png', dpi=300)
