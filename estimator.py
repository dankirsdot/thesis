import math

import numpy as np
import sympy as sym

import casadi as cs

import quadprog
from cvxopt import matrix, solvers

import osqp
from scipy import sparse

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

        self.dp_l = -0.01**2
        self.dp_u =  0.01**2

        self.dt = 2e-2

        # Create an OSQP object
        self.prob = osqp.OSQP()

        # Define problem data
        P = sparse.csc_matrix([0])
        q = np.array([0])
        A = sparse.csc_matrix([1])
        l = np.array([self.p_l])
        u = np.array([self.p_u])

        # Setup workspace
        self.prob.setup(P, q, A, l, u, adaptive_rho=False, verbose=False)


        # horizon length
        if N is not None:
            self.N = N
        else:
            self.N = 100

        self.n = 0

        # const function gains
        self.W = 0.06 / self.N
        self.Gamma = 8e8
        
        # state horizons
        self.theta_h = np.zeros(self.N)
        self.dtheta_h = np.zeros(self.N)
        self.ddtheta_h = np.zeros(self.N)
        self.ddX_h = np.zeros(self.N)

        # Jacobian of the acceleration model
        # with respect to parameter p (r**2)
        theta, dtheta = sym.symbols(r'\theta \dot{\theta}', real=True)
        ddtheta = sym.symbols(r'\ddot{\theta}', real=True)
        L = sym.symbols(r'L', real=True)
        p = sym.symbols(r'p', real=True)

        ddX = (p / sym.sqrt(L**2 - theta**2 * p)) \
            * (theta * ddtheta + dtheta**2 * ((theta**2 * p) / (L**2 - theta**2 * p) + 1))
        ddX_dp = sym.simplify(sym.diff(ddX, p))
        self.J_p = sym.lambdify([theta, dtheta, ddtheta, L, p], ddX_dp)

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
            h_dp = np.array([-self.dp_l / self.dt,
                              self.dp_u / self.dt])
            G = np.append(G, G_dp)
            G = G.reshape(int(len(G)), 1)
            h = np.append(h, h_dp)

            # Define problem data
            # P_new = sparse.csc_matrix([H])
            # q_new = np.array([g])
            # A_new = sparse.csc_matrix([1])
            # l_new = np.array([self.p_l])
            # u_new = np.array([self.p_u])

            # Update osqp problem
            # self.prob.update(Px=sparse.triu(P_new).data, Ax=A_new.data)
            # self.prob.update(q=q_new, l=l_new, u=u_new)

            # Solve the problem
            # res = self.prob.solve()

            # Check solver status
            # if res.info.status != 'solved':
            #    raise ValueError('OSQP did not solve the problem!')
            
            # p_tilde = res.x[0]

            p_tilde = self.quadprog_solve_qp(H, g, G, h)[0]
            # p_tilde = self.cvxopt_solve_qp(H, g, G, h)[0]
            
            self.p = self.p + p_tilde

            r = self.p**0.5
            return r, e

    def get_ddX(self, theta, dtheta, ddtheta, L, p):
        ddX = (p / np.sqrt(L**2 - theta**2 * p)) \
            * (theta * ddtheta + dtheta**2 * ((theta**2 * p) / (L**2 - theta**2 * p) + 1))
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
