import time
import math

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

from can import CANSocket
from gyems import GyemsRMD
from sensors import CANSensors

from setup import Setup

if __name__ == '__main__':
    try:
        sock = CANSocket(interface='can1')
        actuator = GyemsRMD(sock, id=0x141)
        sensors = CANSensors(sock, id=0x01)

        setup = Setup(actuator, sensors)
        
        setup.start()

        t, X, X_d, dX_d, dX_hat, ddX, ddX_hat, \
            dtheta, ddtheta, u, r, F, e = setup.control()
        
        setup.stop()

        t = np.array(t)
        X = np.array(X)
        X_d = np.array(X_d)
        F = np.array(F)
        ddX_accel = np.array(ddX)
        ddX_hat = np.array(ddX_hat)
        ddtheta = np.array(ddtheta)
        e = np.array(e)

        tilde_X = X_d - X
        frequency = 1 / np.diff(t)

        dX = np.diff(X) / np.diff(t)
        ddX = np.diff(dX) / np.diff(t)[:-1]

        butter = signal.butter(10, 20, 'lp', fs=200, output='sos')
        ddtheta_filtered = signal.sosfilt(butter, ddtheta)

        plt.figure(figsize=(7, 3))
        plt.title(r'Linear position $X(t)$')
        plt.plot(t, X, 'r', linewidth=1.5, label=r'$X(t)$', zorder=2)
        plt.plot(t, X_d, 'g', linewidth=1.5, label=r'$X_d(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Contraction $X(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/X.png', dpi=300)

        start = 0
        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction error $\tilde{X}(t)$')
        plt.plot(t, tilde_X, 'r', linewidth=1.5, label=r'$\tilde{X}(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([t[start], t[-1]])
        plt.ylim([np.max(tilde_X[start:]), np.min(tilde_X[start:])])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Contraction error $\tilde{X}(t)$')
        plt.tight_layout()
        plt.legend()
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
        plt.legend()
        plt.savefig('plots/F.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Linear acceleration $\ddot{X}(t)$')
        plt.plot(t, ddX_accel, 'r', linewidth=1.5, label=r'$\ddot{X}_a$', zorder=2)
        # plt.plot(t[:-2], ddX, 'g', linewidth=1.5, label=r'$\ddot{X}$', zorder=1)
        plt.plot(t, ddX_hat, 'b', linewidth=1.5, label=r'$\hat{\ddot{X}}$', zorder=1)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.ylim([-1500, 1500])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Acceleration $\ddot{X}(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/ddX.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Angular acceleration $\ddot{\theta}(t)$')
        plt.plot(t, ddtheta, 'r', linewidth=1.5, label=r'$\ddot{\theta}$', zorder=1)
        plt.plot(t, ddtheta_filtered, 'b', linewidth=1.5, label=r'$\ddot{\theta}_f$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Acceleration $\ddot{\theta}(t)$')
        plt.tight_layout()
        plt.legend()
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
        plt.legend()
        plt.savefig('plots/frequency.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Estimated radius $r(t)$')
        plt.plot(t, r, 'r', linewidth=1.5, label=r'$r(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Radius $r(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/r.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Contraction speed $\dot{X}(t)$')
        plt.plot(t[:-1], dX, 'r', linewidth=1.5, label=r'$\dot{X}(t)$', zorder=2)
        plt.plot(t, dX_hat, 'g', linewidth=1.5, label=r'$\hat{\dot{X}}(t)$', zorder=2)
        plt.plot(t, dX_d, 'b', linewidth=1.5, label=r'$\dot{X}_d(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Speed $\dot{X}(t)$')
        plt.tight_layout()
        plt.legend()
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
        plt.legend()
        plt.savefig('plots/u.png', dpi=300)

        plt.figure(figsize=(7, 3))
        plt.title(r'Acceleration error $e(t)$')
        plt.plot(t, e, 'r', linewidth=1.5, label=r'$e(t)$', zorder=2)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([0, t[-1]])
        plt.xlabel(r'Time $t$')
        plt.ylabel(r'Error $e(t)$')
        plt.tight_layout()
        plt.legend()
        plt.savefig('plots/e.png', dpi=300)

    except Exception as e:
        print(e)
    finally:
        setup.stop()