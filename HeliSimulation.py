import numpy as np
import scipy.integrate
import ModelConstants as mc
from enum import Enum
import time


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
    p_max = 80 / 180.0 * np.pi
    p_min = -p_max
    e_max = 70.0 / 180.0 * np.pi
    e_min = -e_max
    lamb_max = 170.0 / 180.0 * np.pi
    lamb_min = -lamb_max
    eps_p = 0
    eps_e = 0
    eps_lamb = 0

    def __init__(self):
        self.p = LimitType.NO_LIMIT_REACHED
        self.e = LimitType.NO_LIMIT_REACHED
        self.lamb = LimitType.NO_LIMIT_REACHED

def event_pmin(t, x):
    return x[0] - StateLimits.p_min
event_pmin.terminal = True

def event_pmax(t, x):
    return x[0] - StateLimits.p_max
event_pmax.terminal = True

def event_pdt0(t, x):
    return x[6]
event_pdt0.terminal = True

def event_emin(t, x):
    return x[1] - StateLimits.e_min
event_emin.terminal = True

def event_emax(t, x):
    return x[1] - StateLimits.e_max
event_emax.terminal = True

def event_edt0(t, x):
    return x[7]
event_edt0.terminal = True


class HeliSimulation(object):
    def __init__(self, theta1_0, theta2_0, theta3_0, timeStep):
        """Initializes the simulation"""
        self.currentState = np.array([theta1_0, theta2_0, theta3_0, 0, 0, 0, 0, 0, 0])
        self.should_check_limits = True
        self.statLim = StateLimits()
        self.timeStep = timeStep
        self.model_type = ModelType.CENTRIPETAL
        self.currentTime = 0

    # def checkLimits(self):
    #     """handles the limit state machine"""
    #     p, e, lamb, dp, de, dlamb = self.currentState
    #
    #     #pitch limits
    #     if self.limitedStates.p == LimitType.NO_LIMIT_REACHED:
    #         if self.currentState[0] > self.limitedStates.p_max - self.limitedStates.eps_p:
    #             print("[DEBUG] Reached upper limit of p")
    #             self.limitedStates.p = LimitType.UPPER_LIMIT
    #             p = self.limitedStates.p_max
    #             dp = 0
    #         if self.currentState[0] < self.limitedStates.p_min + self.limitedStates.eps_p:
    #             print("[DEBUG] Reached lower limit of p")
    #             self.limitedStates.p = LimitType.LOWER_LIMIT
    #             p = self.limitedStates.p_min
    #             dp = 0
    #     if self.limitedStates.p == LimitType.UPPER_LIMIT:
    #         if p + dp * self.timeStep < self.limitedStates.p_max - self.limitedStates.eps_p:
    #             print("[DEBUG] p is not limited anymore")
    #             self.limitedStates.p = LimitType.NO_LIMIT_REACHED
    #         else:
    #             p = self.limitedStates.p_max
    #             dp = 0
    #     if self.limitedStates.p == LimitType.LOWER_LIMIT:
    #         if p + dp * self.timeStep > self.limitedStates.p_min + self.limitedStates.eps_p:
    #             print("[DEBUG] p is not limited anymore")
    #             self.limitedStates.p = LimitType.NO_LIMIT_REACHED
    #         else:
    #             p = self.limitedStates.p_min
    #             dp = 0
    #     #elevation limits
    #     if self.limitedStates.e == LimitType.NO_LIMIT_REACHED:
    #         if self.currentState[1] > self.limitedStates.e_max - self.limitedStates.eps_e:
    #             print("[DEBUG] Reached upper limit of e")
    #             self.limitedStates.e = LimitType.UPPER_LIMIT
    #             e = self.limitedStates.e_max
    #             de =  0
    #         if self.currentState[1] < self.limitedStates.e_min + self.limitedStates.eps_e:
    #             print("[DEBUG] Reached lower limit of e")
    #             self.limitedStates.e = LimitType.LOWER_LIMIT
    #             e = self.limitedStates.e_min
    #             de = 0
    #     if self.limitedStates.e == LimitType.UPPER_LIMIT:
    #         if e + de * self.timeStep < self.limitedStates.e_max - self.limitedStates.eps_e:
    #             print("[DEBUG] e is not limited anymore")
    #             self.limitedStates.e = LimitType.NO_LIMIT_REACHED
    #         else:
    #             e = self.limitedStates.e_max
    #             de = 0
    #     if self.limitedStates.e == LimitType.LOWER_LIMIT:
    #         if e + de * self.timeStep > self.limitedStates.e_min + self.limitedStates.eps_e:
    #             print("[DEBUG] e is not limited anymore")
    #             self.limitedStates.e = LimitType.NO_LIMIT_REACHED
    #         else:
    #             e = self.limitedStates.e_min
    #             de = 0
    #     #travel angle limits
    #     if self.limitedStates.lamb == LimitType.NO_LIMIT_REACHED:
    #         if self.currentState[2] > self.limitedStates.lamb_max - self.limitedStates.eps_lamb:
    #             print("[DEBUG] Reached upper limit of lambda")
    #             self.limitedStates.lamb = LimitType.UPPER_LIMIT
    #             lamb = self.limitedStates.lamb_max
    #             dlamb = 0
    #         if self.currentState[2] < self.limitedStates.lamb_min + self.limitedStates.eps_lamb:
    #             print("[DEBUG] Reached lower limit of lambda")
    #             self.limitedStates.lamb = LimitType.LOWER_LIMIT
    #             lamb = self.limitedStates.lamb_min
    #             dlamb = 0
    #     if self.limitedStates.lamb == LimitType.UPPER_LIMIT:
    #         if lamb + dlamb * self.timeStep < self.limitedStates.lamb_max - self.limitedStates.eps_lamb:
    #             print("[DEBUG] lambda is not limited anymore")
    #             self.limitedStates.e = LimitType.NO_LIMIT_REACHED
    #         else:
    #             lamb = self.limitedStates.lamb_max
    #             dlamb = 0
    #     if self.limitedStates.lamb == LimitType.LOWER_LIMIT:
    #         if lamb + dlamb * self.timeStep > self.limitedStates.lamb_min + self.limitedStates.eps_lamb:
    #             print("[DEBUG] lambda is not limited anymore")
    #             self.limitedStates.lamb = LimitType.NO_LIMIT_REACHED
    #         else:
    #             lamb = self.limitedStates.lamb_min
    #             dlamb = 0
    #
    #     self.solver.set_initial_value([p, e, lamb, dp, de, dlamb], self.solver.t)
    #     return

    def rhs_old(self, t, x, V_s, V_d):
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

        return np.array([dp, de, dlamb, ddp, dde, ddlamb])

    def rhs(self, t, x, V_s, V_d):
        """Right hand side of the differential equation being integrated"""
        p, e, lamb, dp, de, dlamb, ddp_event, dde_event, ddlamb_event = x

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

        ddp_event = ddp
        dde_event = dde
        ddlamb_event = ddlamb_event

        if self.statLim.p == LimitType.UPPER_LIMIT or self.statLim.p == LimitType.LOWER_LIMIT:
            dp = 0
            ddp = 0

        if self.statLim.e == LimitType.UPPER_LIMIT or self.statLim.e == LimitType.LOWER_LIMIT:
            de = 0
            dde = 0

        return np.array([dp, de, dlamb, ddp, dde, ddlamb, ddp_event, dde_event, ddlamb_event])

    def calcStep(self, V_f, V_b):
        """Returns the state of the system after the next time step
        V_f: voltage of the propeller right at back (of Fig.7) / first endeffector
        V_b: voltage of the propeller right at front (of Fig. 7) / second endeffector"""
        #start = time.time()
        V_s = V_f + V_b
        V_d = V_f - V_b
        # checkLimits is called before integrate because the solver calls the rhs function several times
        # between two discrete steps. if there are state transitions between two steps, the solver
        # will probably react unpredictable
        if self.should_check_limits and 1 == 2:
            self.checkLimits()
        #integrate one time step
        #for adjusting the performance the number of in-between-steps can be changed
        tt = np.linspace(self.currentTime, self.currentTime + self.timeStep, 3)
        sol = scipy.integrate.solve_ivp(lambda t, x: self.rhs(t, x, V_s, V_d),
                                        (self.currentTime, self.currentTime+self.timeStep), self.currentState,
                                        method='RK45', t_eval=tt,  rtol=1e-6, atol=1e-9,
                                        events=[event_pmin, event_pmax,
                                                event_emin, event_emax, event_pdt0, event_edt0])
        #check if there was an event in order to limit the angle states
        ##TODO: Simultion Time must always stop at +timeStep !
        if np.size(sol.t_events) != 0:
            #1. Manage state machine
            if np.size(sol.t_events[0]) != 0:
                print("pmin")
                self.statLim.p = LimitType.LOWER_LIMIT
            if np.size(sol.t_events[1]) != 0:
                print("pmax")
                self.statLim.p = LimitType.UPPER_LIMIT
            if np.size(sol.t_events[2]) != 0:
                print("emin")
                self.statLim.e = LimitType.LOWER_LIMIT
            if np.size(sol.t_events[3]) != 0:
                print("emax")
                self.statLim.e = LimitType.UPPER_LIMIT
            if np.size(sol.t_events[4]) != 0:
                print("p  no limit")
                self.statLim.p = LimitType.NO_LIMIT_REACHED
            if np.size(sol.t_events[5]) != 0:
                print("e  no limit")
                self.statLim.e = LimitType.NO_LIMIT_REACHED

            #2. Simulate until end of time interval
            #Because of all events terminate the simulation it is clear that there only can be
            #one event in the t_events array. Therefore we just have to look up this time value
            self.currentTime =
            sol2 = scipy.integrate.solve_ivp(lambda t, x: self.rhs(t, x, V_s, V_d),
                                            (self.currentTime, self.currentTime + self.timeStep), self.currentState,
                                            method='RK45', t_eval=tt, rtol=1e-6, atol=1e-9,
                                            events=[event_pmin, event_pmax,
                                                    event_emin, event_emax])
        self.currentState = sol.y[:, -1]
        self.currentTime = sol.t[-1]
        #end = time.time()
        #print(end - start)
        return self.currentState[0:6]

    def getCurrentState(self):
        return self.currentState[0:6]

    def getCurrentTime(self):
        return self.currentTime

    def setCurrentStateAndTime(self, state, time=0.0):
        state = np.array(state)
        state = np.append(state, state[3:]) #convert outside state to inside state
        #print("setCurrentStateandTime")
        #print(state)
        self.currentTime = time
        self.currentState = state

    def setModelType(self, modelType):
        self.model_type = modelType