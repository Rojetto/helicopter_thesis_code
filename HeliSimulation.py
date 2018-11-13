import numpy as np
import scipy.integrate
import ModelConstants as mc
from enum import Enum


class ModelType(Enum):
    EASY = 1
    FRICTION = 2
    CENTRIPETAL = 3


class LimitType(Enum):
    NO_LIMIT_REACHED = 0
    UPPER_LIMIT = 1
    LOWER_LIMIT = -1


L_1 = mc.l_p
L_2 = mc.g * (mc.l_c * mc.m_c - 2 * mc.l_h * mc.m_p)
L_3 = mc.l_h
L_4 = mc.l_h

J_p = 2*mc.m_p*mc.l_p**2
J_e = mc.m_c * mc.l_c**2 + 2* mc.m_p*mc.l_h**2
J_l = mc.m_c * mc.l_c**2 + 2 * mc.m_p * (mc.l_h**2 + mc.l_p**2)


class StateLimits(object):
    """
    Helper class just for saving which states have reached their limit at the current time step,
    but also for saving some attributes about the limits
    """
    def __init__(self):
        self.p = LimitType.NO_LIMIT_REACHED
        self.e = LimitType.NO_LIMIT_REACHED
        self.lamb = LimitType.NO_LIMIT_REACHED
        self.p_max = 80 / 180.0 * np.pi
        self.p_min = -self.p_max
        self.e_max = 70.0 / 180.0 * np.pi
        self.e_min = -self.e_max
        self.lamb_max = 170.0 / 180.0 * np.pi
        self.lamb_min = -self.lamb_max
        self.eps_p = 0
        self.eps_e = 0
        self.eps_lamb = 0


class HeliSimulation(object):
    def __init__(self, theta1_0, theta2_0, theta3_0, timeStep):
        """Initializes the simulation"""
        self.currentState = np.array([theta1_0, theta2_0, theta3_0, 0, 0, 0])
        self.should_check_limits = True
        self.limitedStates = StateLimits()
        self.timeStep = timeStep
        self.model_type = ModelType.CENTRIPETAL
        self.solver = scipy.integrate.ode(lambda t, x: self.rhs(t, x, self.V_s, self.V_d))
        self.solver.set_initial_value([theta1_0, theta2_0, theta3_0, 0, 0, 0])
        self.solver.set_integrator('vode', method='adams', rtol=1e-6, atol=1e-9)

    def rhs(self, t, x, V_s, V_d):
        """Right hand side of the differential equation being integrated"""
        p, e, lamb, dp, de, dlamb = x

        # calculate dp, de and dlamb
        if self.model_type == ModelType.EASY:
            ddp = (L_1 / J_p) * V_d
            dde = (L_2/J_e) * np.cos(e) + (L_3/J_e) * np.cos(p) * V_s
            ddlamb = (L_4/J_l) * np.cos(e) * np.sin(p) * V_s

        if self.model_type == ModelType.FRICTION:
            ddp = (L_1 / J_p) * V_d - (mc.d_p / J_p) * dp
            dde = (L_2/J_e) * np.cos(e) + (L_3/J_e) * np.cos(p) * V_s - (mc.d_e / J_e) * de
            ddlamb = (L_4/J_l) * np.cos(e) * np.sin(p) * V_s - (mc.d_l / J_l) * dlamb

        if self.model_type == ModelType.CENTRIPETAL:
            ddp = (L_1 / J_p) * V_d - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de**2 - np.cos(e)**2 * dlamb**2)
            dde = (L_2/J_e) * np.cos(e) + (L_3/J_e) * np.cos(p) * V_s - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dlamb**2
            ddlamb = (L_4/J_l) * np.cos(e) * np.sin(p) * V_s - (mc.d_l / J_l) * dlamb

        return [dp, de, dlamb, ddp, dde, ddlamb]

    def checkLimits(self):
        """handles the limit state machine"""
        p, e, lamb, dp, de, dlamb = self.currentState

        #pitch limits
        if self.limitedStates.p == LimitType.NO_LIMIT_REACHED:
            if self.currentState[0] > self.limitedStates.p_max - self.limitedStates.eps_p:
                print("[DEBUG] Reached upper limit of p")
                self.limitedStates.p = LimitType.UPPER_LIMIT
                p = self.limitedStates.p_max
                dp = 0
            if self.currentState[0] < self.limitedStates.p_min + self.limitedStates.eps_p:
                print("[DEBUG] Reached lower limit of p")
                self.limitedStates.p = LimitType.LOWER_LIMIT
                p = self.limitedStates.p_min
                dp = 0
        if self.limitedStates.p == LimitType.UPPER_LIMIT:
            if p + dp * self.timeStep < self.limitedStates.p_max - self.limitedStates.eps_p:
                print("[DEBUG] p is not limited anymore")
                self.limitedStates.p = LimitType.NO_LIMIT_REACHED
            else:
                p = self.limitedStates.p_max
                dp = 0
        if self.limitedStates.p == LimitType.LOWER_LIMIT:
            if p + dp * self.timeStep > self.limitedStates.p_min + self.limitedStates.eps_p:
                print("[DEBUG] p is not limited anymore")
                self.limitedStates.p = LimitType.NO_LIMIT_REACHED
            else:
                p = self.limitedStates.p_min
                dp = 0
        #elevation limits
        if self.limitedStates.e == LimitType.NO_LIMIT_REACHED:
            if self.currentState[1] > self.limitedStates.e_max - self.limitedStates.eps_e:
                print("[DEBUG] Reached upper limit of e")
                self.limitedStates.e = LimitType.UPPER_LIMIT
                e = self.limitedStates.e_max
                de =  0
            if self.currentState[1] < self.limitedStates.e_min + self.limitedStates.eps_e:
                print("[DEBUG] Reached lower limit of e")
                self.limitedStates.e = LimitType.LOWER_LIMIT
                e = self.limitedStates.e_min
                de = 0
        if self.limitedStates.e == LimitType.UPPER_LIMIT:
            if e + de * self.timeStep < self.limitedStates.e_max - self.limitedStates.eps_e:
                print("[DEBUG] e is not limited anymore")
                self.limitedStates.e = LimitType.NO_LIMIT_REACHED
            else:
                e = self.limitedStates.e_max
                de = 0
        if self.limitedStates.e == LimitType.LOWER_LIMIT:
            if e + de * self.timeStep > self.limitedStates.e_min + self.limitedStates.eps_e:
                print("[DEBUG] e is not limited anymore")
                self.limitedStates.e = LimitType.NO_LIMIT_REACHED
            else:
                e = self.limitedStates.e_min
                de = 0
        #travel angle limits
        if self.limitedStates.lamb == LimitType.NO_LIMIT_REACHED:
            if self.currentState[2] > self.limitedStates.lamb_max - self.limitedStates.eps_lamb:
                print("[DEBUG] Reached upper limit of lambda")
                self.limitedStates.lamb = LimitType.UPPER_LIMIT
                lamb = self.limitedStates.lamb_max
                dlamb = 0
            if self.currentState[2] < self.limitedStates.lamb_min + self.limitedStates.eps_lamb:
                print("[DEBUG] Reached lower limit of lambda")
                self.limitedStates.lamb = LimitType.LOWER_LIMIT
                lamb = self.limitedStates.lamb_min
                dlamb = 0
        if self.limitedStates.lamb == LimitType.UPPER_LIMIT:
            if lamb + dlamb * self.timeStep < self.limitedStates.lamb_max - self.limitedStates.eps_lamb:
                print("[DEBUG] lambda is not limited anymore")
                self.limitedStates.e = LimitType.NO_LIMIT_REACHED
            else:
                lamb = self.limitedStates.lamb_max
                dlamb = 0
        if self.limitedStates.lamb == LimitType.LOWER_LIMIT:
            if lamb + dlamb * self.timeStep > self.limitedStates.lamb_min + self.limitedStates.eps_lamb:
                print("[DEBUG] lambda is not limited anymore")
                self.limitedStates.lamb = LimitType.NO_LIMIT_REACHED
            else:
                lamb = self.limitedStates.lamb_min
                dlamb = 0

        self.solver.set_initial_value([p, e, lamb, dp, de, dlamb], self.solver.t)
        return

    def calcStep(self, V_f, V_b):
        """Returns the state of the system after the next time step
        V_f: voltage of the propeller right at back (of Fig.7) / first endeffector
        V_b: voltage of the propeller right at front (of Fig. 7) / second endeffector"""
        self.V_s = V_f + V_b
        self.V_d = V_f - V_b
        # checkLimits is called before integrate because the solver calls the rhs function several times
        # between two discrete steps. if there are state transitions between two steps, the solver
        # will probably react unpredictable
        if self.should_check_limits:
            self.checkLimits()
        self.currentState = self.solver.integrate(self.solver.t + self.timeStep)
        return self.currentState

    def getCurrentState(self):
        return self.currentState #yes, this is technically seen unnecessary, but accessing 'private' member objects from outside is rather ugly

    def getCurrentTime(self):
        return self.solver.t

    def setCurrentStateAndTime(self, state, time=0.0):
        self.currentState = np.array(state)
        self.solver.set_initial_value(self.currentState, time)

    def setModelType(self, modelType):
        self.model_type = modelType