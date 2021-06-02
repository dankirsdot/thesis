import math

import numpy as np
from sympy import symbols, sqrt, diff, simplify, lambdify

import quadprog

class Estimator:
    """ 
        TODO: add description
    """

    def __init__(self, L, r):
        self.L = L
        self.psi = 1 / r**2
        
        self.Gamma = 1e-3

        self.n = 0
        self.sum_HH = 0

    def update(self, theta, X):
        self.n += 1

        H = 2*X*self.L - X**2
        self.sum_HH += H*H

        R = self.sum_HH / self.n
        inv_R = 1 / R

        e = theta**2 - H*self.psi

        self.psi = self.psi + self.Gamma * inv_R * H * e

        r = math.sqrt(1 / self.psi)
        return r

class EstimatorMHE:
    """ 
        TODO: add description
    """

    def __init__(self, L, r, N=None):
        self.L = L
        self.p = r**2

        # parameter constraints
        self.p_l = 0.6**2
        self.p_u = 0.9**2

        self.dp_l= -0.01**2
        self.dp_u = 0.01**2

        # horizon length
        if N is not None:
            self.N = N
        else:
            self.N = 100

        self.n = 0

        # const function gains
        self.W = 0.0006
        self.Gamma = 8e9
        
        # state horizons
        self.theta_h = np.zeros(self.N)
        self.dtheta_h = np.zeros(self.N)
        self.ddtheta_h = np.zeros(self.N)
        self.ddX_h = np.zeros(self.N)

        # Jacobian of the acceleration model
        # with respect to parameter p (r**2)
        theta, dtheta = symbols(r'\theta \dot{\theta}', real=True)
        ddtheta = symbols(r'\ddot{\theta}', real=True)
        L = symbols(r'L', real=True)
        p = symbols(r'p', real=True)

        ddX = p / sqrt(L**2 - theta**2 * p) \
            * (theta * ddtheta + dtheta**2 * (theta**2 * p / (L**2 - theta**2 * p) + 1))
        ddX_dp = simplify(diff(ddX, p))
        self.J_p = lambdify([theta, dtheta, ddtheta, L, p], ddX_dp)

    # def J_p(self, theta, dtheta, ddtheta, L, p):
    #     j_p = self.j_p(theta, dtheta, ddtheta, L, p)
    #     return np.array(j_p)

    def estimate(self, theta, dtheta, ddtheta, ddX):
        # shift horizon
        self.theta_h = np.roll(self.theta_h, shift=-1)
        self.dtheta_h = np.roll(self.dtheta_h, shift=-1)
        self.ddtheta_h = np.roll(self.ddtheta_h, shift=-1)
        self.ddX_h = np.roll(self.ddX_h, shift=1)

        # add new data
        self.theta_h[-1] = theta
        self.dtheta_h[-1] = dtheta
        self.ddtheta_h[-1] = ddtheta
        self.ddX_h[-1] = ddX

        if self.n < self.N - 1:
            self.n += 1
            r = self.p**0.5
            e = 0
            return r, e
        else:
            H, g, e = self.get_qp()

            # self.p = self.p + (1 / H) * g

            H = np.array([[H]])
            g = np.array([g])

            G = np.array([[-1.], [1.]])

            h = np.array([-self.p_l + self.p,
                           self.p_u - self.p])

            p_tilde = self.quadprog_solve_qp(H, g, G, h)[0]
            self.p = self.p + p_tilde

            r = self.p**0.5
            return r, e

    def get_ddX(self, theta, dtheta, ddtheta, L, p):
        ddX = p / np.sqrt(L**2 - theta**2 * p) \
            * (theta * ddtheta + dtheta**2 * (theta**2 * p / (L**2 - theta**2 * p) + 1))
        return ddX

    def get_qp(self):
        J_p = self.J_p(self.theta_h, self.dtheta_h, self.ddtheta_h, self.L, self.p)
        ddX_hat = self.get_ddX(self.theta_h, self.dtheta_h, self.ddtheta_h, self.L, self.p)
        H = np.dot(self.W * J_p, J_p) + self.Gamma
        
        e = ddX_hat - self.ddX_h
        g = np.dot(self.W * e, J_p)
        return H, g, e[-1]

    def quadprog_solve_qp(self, P, q, G=None, h=None, A=None, b=None):
        qp_G = .5 * (P + P.T)   # make sure P is symmetric
        qp_a = -q
        if A is not None:
            qp_C = -np.vstack([A, G]).T
            qp_b = -np.hstack([b, h])
            meq = A.shape[0]
        else:  # no equality constraint
            qp_C = -G.T
            qp_b = -h
            meq = 0
        return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]



class EstimatorMHE_both:
    """ 
        TODO: add description
    """

    def __init__(self, L, r, N=None):
        self.p_1 = r**2
        self.p_2 = L**2
        
        # parameter constraints
        self.p_l = np.array([0.6**2, 200**2])
        self.p_u = np.array([0.9**2, 220**2])

        self.dp_l= np.array([-0.01**2, -0.5**2])
        self.dp_u = np.array([0.01**2, 0.5**2])

        # horizon length
        if N is not None:
            self.N = N
        else:
            self.N = 100

        self.n = 0

        # const function gains
        self.W = 0.06
        self.Gamma = np.diag([8e8, 1e9])
        
        # state horizons
        self.theta_h = np.zeros(self.N)
        self.dtheta_h = np.zeros(self.N)
        self.ddtheta_h = np.zeros(self.N)
        self.ddX_h = np.zeros(self.N)

        # Jacobian of the acceleration model
        # with respect to parameter p (r**2)
        theta, dtheta = symbols(r'\theta \dot{\theta}', real=True)
        ddtheta = symbols(r'\ddot{\theta}', real=True)
        p_1, p_2 = symbols(r'p_1, p_2', real=True)

        ddX = p_1 / sqrt(p_2 - theta**2 * p_1) \
            * (theta * ddtheta + dtheta**2 * (theta**2 * p_1 / (p_2 - theta**2 * p_1) + 1))
        ddX_dp_1 = simplify(diff(ddX, p_1))
        ddX_dp_2 = simplify(diff(ddX, p_2))
        self.ddX_dp_1 = lambdify([theta, dtheta, ddtheta, p_1, p_2], ddX_dp_1)
        self.ddX_dp_2 = lambdify([theta, dtheta, ddtheta, p_1, p_2], ddX_dp_2)

    def J_p(self, theta, dtheta, ddtheta):
        ddX_dp_1 = self.ddX_dp_1(theta, dtheta, ddtheta, self.p_1, self.p_2)
        ddX_dp_2 = self.ddX_dp_1(theta, dtheta, ddtheta, self.p_1, self.p_2)
        j_p = np.array([ddX_dp_1, ddX_dp_2])
        return j_p

    def estimate(self, theta, dtheta, ddtheta, ddX):
        # shift horizon
        self.theta_h = np.roll(self.theta_h, shift=-1)
        self.dtheta_h = np.roll(self.dtheta_h, shift=-1)
        self.ddtheta_h = np.roll(self.ddtheta_h, shift=-1)
        self.ddX_h = np.roll(self.ddX_h, shift=1)

        # add new data
        self.theta_h[-1] = theta
        self.dtheta_h[-1] = dtheta
        self.ddtheta_h[-1] = ddtheta
        self.ddX_h[-1] = ddX

        if self.n < self.N - 1:
            self.n += 1
            r = self.p_1**0.5
            L = self.p_2**0.5
            e = 0
            return r, L, e
        else:
            H, g, e = self.get_qp()

            tilde_p = np.dot(np.linalg.inv(H), -g)
            self.p_1 = self.p_1 + tilde_p[0]
            self.p_2 = self.p_2 + tilde_p[1]

            H = np.array(H)
            g = np.array(g)

            n = 2
            p_0 = np.array([self.p_1, self.p_2])

            A, b = [], []
            A_p = np.concatenate((-np.eye(n),
                                   np.eye(n)))
            b_p = np.concatenate((-np.array(self.p_l) + p_0,
                                   np.array(self.p_u) - p_0))
            A = np.append(A, A_p)
            A = A.reshape(int(len(A) / n), n)
            b = np.append(b, b_p)

            # A_dp = np.concatenate((-np.eye(n),
            #                         np.eye(n)))
            # b_dp = np.concatenate((-self.dp_l,
            #                         self.dp_u))
            # A = np.append(A, A_dp)
            # A = A.reshape(int(len(A) / n), n)
            # b = np.append(b, b_dp)

            # tilde_p = self.quadprog_solve_qp(H, g, A, b)
            self.p_1 = self.p_1 + tilde_p[0]
            self.p_2 = self.p_2 + tilde_p[1]

            r = self.p_1**0.5
            L = self.p_2**0.5
            print(r, L)
            return r, L, e

    def get_ddX(self, theta, dtheta, ddtheta, p_1, p_2):
        ddX = p_1 / np.sqrt(p_2 - theta**2 * p_1) \
            * (theta * ddtheta + dtheta**2 * (theta**2 * p_1 / (p_2 - theta**2 * p_1) + 1))
        return ddX

    def get_qp(self):
        J_p = self.J_p(self.theta_h, self.dtheta_h, self.ddtheta_h).T
        ddX_hat = self.get_ddX(self.theta_h, self.dtheta_h, self.ddtheta_h, self.p_1, self.p_2)
        H = np.dot(np.dot(J_p.T, self.W), J_p) + self.Gamma

        e = ddX_hat - self.ddX_h
        g = np.dot(np.dot(e.T, self.W), J_p)
        return H, g, e[-1]

    def quadprog_solve_qp(self, P, q, G=None, h=None, A=None, b=None):
        qp_G = .5 * (P + P.T)   # make sure P is symmetric
        qp_a = -q
        if A is not None:
            qp_C = -np.vstack([A, G]).T
            qp_b = -np.hstack([b, h])
            meq = A.shape[0]
        else:  # no equality constraint
            qp_C = -G.T
            qp_b = -h
            meq = 0
        return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]