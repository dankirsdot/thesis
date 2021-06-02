import math

class chirp_trajectory:
    """ 
        TODO: add description
    """

    def __init__(self, t_0, t_f, omega_0, omega_f, x_0, x_max):
        
        self.t_0 = t_0
        self.t_f = t_f

        self.omega_0 = omega_0
        self.omega_f = omega_f
        self.domega = (omega_f - omega_0)/(t_f - t_0)

        self.x_0 = x_0
        self.x_max = x_max

    def get(self, t):
        omega = self.omega_0 + self.domega * t
        
        x = (1/2) * self.x_max * (1 - math.cos(omega * t)) + self.x_0
        dx = (1/2) * self.x_max * (omega + t * self.domega) \
            * math.sin(omega * t)

        return x, dx