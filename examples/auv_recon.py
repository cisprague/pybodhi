# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

'''
# Simple vehicle reconnaissance

Consider a vehicle tasked with travelling through a squence of waypoints and
returning to its origin upon completion.
Behaviour trees are well suited to reactively enforce such a mission.

In this example, consider and autonomous underwater vehicle (AUV) tasked with
travelling through a series of waypoints and returning to its origin upon
completion.
'''

import numpy as np, matplotlib.pyplot as plt, sys; sys.path.append('../')
from pybodhi import Tree, Sequence, Fallback
from scipy.integrate import ode

class AUV_Recon(Tree):

    def __init__(self):

        # positional bound [m]
        bound = 50

        # number of waypoints
        self.N = 6

        # origin [m]
        self.O = np.random.uniform(-bound, bound, 2)

        # waypoints [m]
        self.W = np.random.uniform(-bound, bound, (self.N, 2))

        # mission completion
        self.alpha = 0

        # waypoint index
        self.beta = 0

        # back to origin!
        self.gamma = 0

        # waypoint tolerance
        self.eps = 1e-3

        # behaviour tree nodes
        node = Fallback([self.terminal_beta, self.update_beta])
        node = Sequence([node, self.update_alpha])
        node = Sequence([Fallback([self.at_waypoint, self.to_waypoint]), node])
        node = Fallback([self.complete, node])
        node = Sequence([node, Fallback([self.at_origin, self.to_origin]), self.done])

        # initialise behaviour tree
        Tree.__init__(self, node)

        # max thrust
        self.thrust = 10

        # mass
        self.mass = 100

        # planaform area perpendicular to fluid flow
        self.A = 1

        # coefficient of drag of sphere
        self.CD = 0.47

        # fluid density of sea water [kg/m^3]
        self.rho = 1025

        # use AUV dynamics shown below
        self.sys = ode(self.eom)

        # use Runge-Kutta 8(5,3) adaptive step-size integrator
        self.sys.set_integrator('dop853', atol=1e-10, rtol=1e-10, nsteps=10000)

        # record times, states, and query behaviour tree
        self.sys.set_solout(self.sync)

    def terminal_beta(self):
        return int(self.beta is self.N - 1)

    def update_beta(self):
        self.beta += 1
        return 2

    def update_alpha(self):
        self.alpha = 1
        return 2

    def at_waypoint(self):
        return int(np.linalg.norm(self.states[-1, :2] - self.W[self.beta]) <= self.eps)

    def to_waypoint(self):
        self.target = self.W[self.beta]
        return 2

    def complete(self):
        return int(self.alpha)

    def at_origin(self):
        return int(np.linalg.norm(self.states[-1, :2] - self.O) <= self.eps)

    def to_origin(self):
        self.target = self.O
        return 2

    def done(self):
        self.gamma = 1
        return 2

    def eom(self, time, state):

        # extract state
        x, y, vx, vy = state

        # target relative position
        prel = self.target - np.array([x, y], float)

        # target direction
        drel = prel/np.linalg.norm(prel)

        # make control
        ut = 1
        ux, uy = drel

        # common subexpression elimination
        x0 = 1/self.mass
        x1 = self.thrust*ut*x0
        x2 = self.A*self.CD*self.rho*x0*np.sqrt(vx**2 + vy**2)/2

        # return state transition
        ds = np.array([vx, vy, ux*x1 - vx*x2, uy*x1 - vy*x2], float)
        return ds

    def sync(self, time, state):

        # record times and states
        self.times = np.append(self.times, time)
        self.states = np.vstack((self.states, state))

        # query behaviour tree
        self()

        # look for behaviour tree break
        if self.gamma:
            return -1

    def simulate(self, t=5000):

        # initialise records
        self.times = np.empty((1, 0), float)
        self.states = np.empty((0, 4), float)

        # initial state and time
        s0, t0 = np.hstack((self.O, [0, 0])), 0

        # initial target [m]
        self.target = self.W[self.beta]
        self.sys.set_initial_value(s0, t0)
        self.sys.integrate(t)

    def plot(self, ax=None):

        # plot trajectory
        fig1, ax1 = plt.subplots(1)
        ax1.plot(tree.W[:,0], tree.W[:,1], "k.")
        ax1.plot(tree.O[0], tree.O[1], "kx")
        plt.legend(["Waypoints", "Origin"])
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")
        ax1.plot(tree.states[:, 0], tree.states[:, 1], "k--")
        ax1.set_aspect('equal')

        # plot behaviour tree responses
        fig2, ax2 = plt.subplots(1)
        Tree.plot(self, ax2)

        return fig1, ax1, fig2, ax2



if __name__ == "__main__":

    tree = AUV_Recon()
    tree.simulate()
    fig1, ax1, fig2, ax2 = tree.plot()
    fig1.savefig("trajectory.png", dpi=500, bbox_inches="tight")
    fig2.savefig("responses.png", dpi=500, bbox_inches="tight")
