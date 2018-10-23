import numpy as np
import scipy.integrate
import ModelConstants as mc

L_1 = 1
L_2 = 1
L_3 = 1
L_4 = 1

J_p = 2*mc.m_p*mc.l_p**2
J_e = mc.m_c_min * mc.l_c**2 + 2* mc.m_p*mc.l_h**2
J_l = mc.m_c_min * mc.l_c**2 + 2 * mc.m_p * (mc.l_h**2 + mc.l_p**2)

class HeliSimulation(object):
    def __init__(self, theta1_0, theta2_0, theta3_0, timeStep):
        """Initializes the simulation"""
        self.currentState = np.array([theta1_0, theta2_0, theta3_0, 0, 0, 0])
        self.timeStep = timeStep
        self.solver = scipy.integrate.ode(lambda t, x: self.rhs(t, x, self.V_s, self.V_d))
        self.solver.set_initial_value([theta1_0, theta2_0, theta3_0, 0, 0, 0])
        self.solver.set_integrator('vode', method='adams', rtol=1e-6, atol=1e-9)

    def rhs(self, t, x, V_s, V_d):
        """Right hand side of the differential equation being integrated"""
        p, e, lamb, dp, de, dlamb = x

        # calculate dp, de and dlamb
        ddp = (L_1 / J_p) * V_d
        dde = (L_2/J_e) * np.cos(e) + (L_3/J_e) * np.sin(p) * V_s
        ddlamb = (L_4/J_l) * np.cos(e) * np.sin(p) * V_s
        return [dp, de, dlamb, ddp, dde, ddlamb]

    def calcStep(self, V_f, V_b):
        """Returns the state of the system after the next time step
        V_f: voltage of the propeller right at back (of Fig.7) / first endeffector
        V_b: voltage of the propeller right at front (of Fig. 7) / second endeffector"""
        self.V_s = V_f + V_b
        self.V_d = V_f - V_b
        self.currentState = self.solver.integrate(self.solver.t + self.timeStep)
        return self.currentState

    def getCurrentState(self):
        return self.currentState #yes, this is technically seen unnecessary, but accessing 'private' member objects from outside is rather ugly

    def getCurrentTime(self):
        return self.solver.t

    def setCurrentState(self, state):
        self.currentState = np.array(state)
        self.solver.set_initial_value(self.currentState, self.solver.t)