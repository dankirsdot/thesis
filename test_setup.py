import time
import math

import numpy as np
import matplotlib.pyplot as plt

from can import CANSocket
from gyems import GyemsRMD
from sensors import CANSensors

from setup import Setup

if __name__ == '__main__':
    try:
        sock_0 = CANSocket(interface='can0')
        sensors = CANSensors(sock_0, id=0x01)

        sock_1 = CANSocket(interface='can1')
        actuator = GyemsRMD(sock_1, id=0x141)

        setup = Setup(actuator, sensors)
        
        turns = 10
        setup.init(turns)

        omega_0 = 2 * math.pi
        omega_f = 2 * math.pi
        t_f = 100.0
        X_max = 20.0

        t, X, X_d, X_hat, dX, dX_d, dX_hat, \
        ddX_accel, ddX_hat, theta, dtheta,  ddtheta, \
        u, r, F, e, r_mhe, e_mhe = setup.start(omega_0, omega_f, t_f, X_max)
        
        r_mhe = np.array(r_mhe)
        e_mhe = np.array(e_mhe)

        t = np.array(t)
        X = np.array(X)
        X_d = np.array(X_d)
        dX = np.array(dX)
        X_hat = np.array(X_hat)
        F = np.array(F)
        ddX_accel = np.array(ddX_accel)
        ddX_hat = np.array(ddX_hat)
        ddtheta = np.array(ddtheta)
        e = np.array(e)
        theta = np.array(theta)

        tilde_X = X_d - X
        frequency = 1 / np.diff(t)

        ddX = np.diff(dX[:-1]) / np.diff(t)[:-1]

        data = np.stack([t, X, X_d, dX, dX_d, dX_hat, \
                         ddX_accel, ddX_hat, theta, dtheta,  ddtheta, \
                         u, r, F, e]).T
        np.savetxt("data.csv", data, delimiter=",")

        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction $X(t)$')
        # plt.plot(t, X, 'r', linewidth=1.5, label=r'$X(t)$', zorder=2)
        plt.scatter(t, X, color='white', s=40.0, linewidths=0.2, edgecolors='black', zorder=1, label=r'$X(t)$')
        plt.plot(t, X_d, 'r', linewidth=1.5, label=r'$X_d(t)$', zorder=3)
        plt.plot(t, X_hat, 'g', linewidth=1.5, label=r'$\hat{X}(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'$X(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/X.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Angular position $X(t)$')
        plt.plot(t, theta, 'r', linewidth=1.5, label=r'$\theta(t)$', zorder=1)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Angle $\theta(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/theta.png', dpi=300)

        start = 0
        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction error $\tilde{X}(t)$')
        plt.plot(t, tilde_X, 'r', linewidth=1.5, label=r'$\tilde{X}(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([t[start], t[-1]])
        plt.ylim([np.max(tilde_X[start:]), np.min(tilde_X[start:])])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'$\tilde{X}(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/tilde_X.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Force $F(t)$')
        plt.plot(t, F, 'r', linewidth=1.5, label=r'$F(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Force $F(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/F.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Linear acceleration $\ddot{X}(t)$')
        plt.plot(t, ddX_accel, 'r', linewidth=1.5, label=r'$\ddot{X}_a$', zorder=1)
        plt.plot(t, ddX_hat, 'g', linewidth=1.5, label=r'$\hat{\ddot{X}}$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        # plt.ylim([-1500, 1500])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Acceleration $\ddot{X}(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/ddX.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Angular acceleration $\ddot{\theta}(t)$')
        plt.plot(t, ddtheta, 'r', linewidth=1.5, label=r'$\ddot{\theta}$', zorder=1)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Aself.X_hat_lceleration $\ddot{\theta}(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/ddtheta.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Frequency $\nu(t)$')
        plt.plot(t[:-1], frequency, 'r', linewidth=1.5, label=r'$\nu(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Frequency $\nu(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/frequency.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Estimated radius $r(t)$')
        plt.plot(t, r, 'r', linewidth=1.5, label=r'$r(t)$', zorder=2)
        plt.plot(t, r_mhe, 'g', linewidth=1.5, label=r'$r_{mhe}(t)$', zorder=1)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'$r(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/r.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction speed $\dot{X}(t)$')
        # plt.plot(t, dX, 'r', linewidth=1.5, label=r'$\dot{X}(t)$', zorder=2)
        plt.scatter(t, dX, color='white', s=40.0, linewidths=0.2, edgecolors='black', zorder=1, label=r'$\dot{X}(t)$')
        plt.plot(t, dX_d, 'r', linewidth=1.5, label=r'$\dot{X}_d(t)$', zorder=3)
        plt.plot(t, dX_hat, 'g', linewidth=1.5, label=r'$\hat{\dot{X}}(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'$\dot{X}(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/dX.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Motor speed $\dot{\theta}(t)$')
        plt.plot(t, dtheta, 'r', linewidth=1.5, label=r'$\dot{\theta}(t)$', zorder=2)
        plt.plot(t, u, 'g', linewidth=1.5, label=r'$u(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Speed $\dot{\theta}(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/u.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Acceleration error $e(t)$')
        plt.plot(t, e, 'r', linewidth=1.5, label=r'$e(t)$', zorder=1)
        plt.plot(t, e_mhe, 'g', linewidth=1.5, label=r'$e_{mhe}(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Error $e(t)$')
        plt.tight_layout()
        plt.legend(loc='lower right')
        plt.savefig('plots/e.png', dpi=300)

    except Exception as e:
        print(e)
    finally:
        setup.to_zero()
