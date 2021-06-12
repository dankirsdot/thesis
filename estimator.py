import math

import numpy as np
from sympy import symbols, sqrt, diff, simplify, lambdify

import quadprog
from cvxopt import matrix, solvers

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
        self.W = 0.06
        self.Gamma = 8e8
        
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

            H = np.array([[H]])
            g = np.array([g])

            G, h = [], []

            G_p = np.array([[-1.], [1.]])
            h_p = np.array([-self.p_l + self.p,
                           self.p_u - self.p])
            G = np.append(G, G_p)
            G = G.reshape(int(len(G)), 1)
            h = np.append(h, h_p)

            G_dp = np.array([[-1.], [1.]])
            h_dp = np.array([-self.dp_l,
                           self.dp_u])
            G = np.append(G, G_dp)
            G = G.reshape(int(len(G)), 1)
            h = np.append(h, h_p)

            p_tilde = self.quadprog_solve_qp(H, g, G, h)[0]
            # p_tilde = self.cvxopt_solve_qp(H, g, G, h)[0]
            
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

    def cvxopt_solve_qp(self, P, q, G=None, h=None, A=None, b=None):
        """
        This function is wrap on CVXOPT solver for a 'n'-dimensional QP problem with 'm' - linear equality and 'k' - linear
        inequality constraints, written in following form:

            Minimize:
                    0.5 x.T * P * x + q.T * x
            Subject to:
                    G * x <= h - linear
                    A * x == b

        :param P: {n by n} PD matrix of quadratic part of problem.
        :param q: {n by 1} linear part of problem.
        :param G: {k by n} Matrix of linear inequality constraints
        :param h: {k by 1} Free vector for linear inequality constraints
        :param A: {m by n} Matrix of linear equality constraints
        :param b: {m by 1} Free vector for linear equality constraints
        :param progress: either show the progress of CVXOPT quadratic solver or not (True/False)

        :return: x_opt: n by 1 solution of problem.
        """
        solvers.options['show_progress'] = False

        P = .5 * (P + P.T)  # make sure P is symmetric

        args = [matrix(P), matrix(q)]

        if G is not None:
            args.extend([matrix(G), matrix(h)])
            if A is not None:
                args.extend([matrix(A), matrix(b)])
        sol = solvers.qp(*args)
        # if 'optimal' not in sol['status']:
        #     return None
        x_opt = np.array(sol['x']).reshape((P.shape[1],))
        return x_opt

class EstimatorMHE_new:
    """ 
        TODO: add description
    """

    def __init__(self, L, r, N=None):
        self.L = L
        self.p = r**2

        # parameter constraints
        self.p_l = 0.0**2
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
        self.W = 0.06
        self.Gamma = 8e4
        
        # state horizons
        self.theta_h = np.zeros(self.N + 1)
        self.dtheta_h = np.zeros(self.N + 1)
        self.ddX_h = np.zeros(self.N)
        self.dt = np.zeros(self.N)

        # Jacobian of the jacobian of TSA model
        # with respect to parameter p (r**2)
        theta, dtheta = symbols(r'\theta \dot{\theta}', real=True)
        L = symbols(r'L', real=True)
        p = symbols(r'p', real=True)

        sym_J = (theta * p) / sqrt(L**2 - theta**2 * p)
        sym_J_diff = simplify(diff(sym_J, p))
        self.lambd_J_diff = lambdify([theta, L, p], sym_J_diff)

    def estimate(self, theta, dtheta, ddX, dt):
        # shift horizon
        self.theta_h = np.roll(self.theta_h, shift=-1)
        self.dtheta_h = np.roll(self.dtheta_h, shift=-1)
        self.ddX_h = np.roll(self.ddX_h, shift=1)
        self.dt = np.roll(self.dt, shift=1)

        # add new data
        self.theta_h[-1] = theta
        self.dtheta_h[-1] = dtheta
        self.ddX_h[-1] = ddX
        self.dt[-1] = dt

        if self.n < self.N:
            self.n += 1
            r = self.p**0.5
            e = 0
            ddX_hat = 0
            return r, e, ddX_hat
        else:
            H, g, e, ddX_hat = self.get_qp()

            H = np.array([[H]])
            g = np.array([g])

            G, h = [], []

            G_p = np.array([[-1.], [1.]])
            h_p = np.array([-self.p_l + self.p,
                           self.p_u - self.p])
            G = np.append(G, G_p)
            G = G.reshape(int(len(G)), 1)
            h = np.append(h, h_p)

            G_dp = np.array([[-1.], [1.]])
            h_dp = np.array([-self.dp_l,
                              self.dp_u])
            G = np.append(G, G_dp)
            G = G.reshape(int(len(G)), 1)
            h = np.append(h, h_p)

            p_tilde = self.quadprog_solve_qp(H, g, G, h)[0]
            # p_tilde = self.cvxopt_solve_qp(H, g, G, h)[0]
            
            self.p = self.p + p_tilde

            r = self.p**0.5
            return r, e, ddX_hat

    def get_J(self, theta, L, p):
        return (theta * p) / np.sqrt(L**2 - theta**2 * p)

    def get_ddX(self, theta, dtheta, theta_p, dtheta_p, L, p, dt):
        J = self.get_J(theta, L, p)
        J_p = self.get_J(theta_p, L, p)
        ddX = (np.multiply(J, dtheta) - \
               np.multiply(J_p, dtheta_p)) / dt
        return ddX

    def get_ddX_dt(self, theta, dtheta, theta_p, dtheta_p, L, p):
        J = self.get_J(theta, L, p)
        J_p = self.get_J(theta_p, L, p)
        ddX_dt = np.multiply(J, dtheta) - np.multiply(J_p, dtheta_p)
        return ddX_dt

    def get_qp(self):
        theta = self.theta_h[1:]
        dtheta = self.dtheta_h[1:]

        theta_p = self.theta_h[:-1]
        dtheta_p = self.dtheta_h[:-1]

        J_X = (np.multiply(self.lambd_J_diff(theta, self.L, self.p), dtheta) - \
               np.multiply(self.lambd_J_diff(theta_p, self.L, self.p), dtheta_p))

        ddX_dt_hat = self.get_ddX_dt(theta, dtheta, theta_p, dtheta_p, self.L, self.p)
        H = np.dot(self.W * J_X, J_X) + self.Gamma
        
        e = ddX_dt_hat - np.multiply(self.ddX_h, self.dt)
        g = np.dot(self.W * e, J_X)
        return H, g, e[-1], ddX_dt_hat[-1] / self.dt[-1]

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

    def cvxopt_solve_qp(self, P, q, G=None, h=None, A=None, b=None):
        """
        This function is wrap on CVXOPT solver for a 'n'-dimensional QP problem with 'm' - linear equality and 'k' - linear
        inequality constraints, written in following form:

            Minimize:
                    0.5 x.T * P * x + q.T * x
            Subject to:
                    G * x <= h - linear
                    A * x == b

        :param P: {n by n} PD matrix of quadratic part of problem.
        :param q: {n by 1} linear part of problem.
        :param G: {k by n} Matrix of linear inequality constraints
        :param h: {k by 1} Free vector for linear inequality constraints
        :param A: {m by n} Matrix of linear equality constraints
        :param b: {m by 1} Free vector for linear equality constraints
        :param progress: either show the progress of CVXOPT quadratic solver or not (True/False)

        :return: x_opt: n by 1 solution of problem.
        """
        solvers.options['show_progress'] = False

        P = .5 * (P + P.T)  # make sure P is symmetric

        args = [matrix(P), matrix(q)]

        if G is not None:
            args.extend([matrix(G), matrix(h)])
            if A is not None:
                args.extend([matrix(A), matrix(b)])
        sol = solvers.qp(*args)
        # if 'optimal' not in sol['status']:
        #     return None
        x_opt = np.array(sol['x']).reshape((P.shape[1],))
        return x_opt