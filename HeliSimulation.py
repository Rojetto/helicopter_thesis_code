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
    lamb_max = 120 / 180.0 * np.pi
    lamb_min = -lamb_max
    eps_p = 0
    eps_e = 0
    eps_lamb = 0

    def __init__(self):
        self.p = LimitType.NO_LIMIT_REACHED
        self.e = LimitType.NO_LIMIT_REACHED
        self.lamb = LimitType.NO_LIMIT_REACHED


class EventParams(object):
    V_s = -1
    V_d = -1
    model_type = -1


def event_pmin(t, x):
    return x[0] - StateLimits.p_min
event_pmin.terminal = True


def event_pmax(t, x):
    return x[0] - StateLimits.p_max
event_pmax.terminal = True


def event_pdt0(t, x):
    """Checks if the second derivative has crossed zero."""
    p, e, lamb, dp, de, dlamb, dp_event, de_event, dlamb_event = x
    if EventParams.model_type == ModelType.EASY:
        ddp = (L_1 / J_p) * EventParams.V_d
    elif EventParams.model_type == ModelType.FRICTION:
        ddp = (L_1 / J_p) * EventParams.V_d - (mc.d_p / J_p) * dp
    elif EventParams.model_type == ModelType.CENTRIPETAL:
        ddp = ((L_1 / J_p) * EventParams.V_d - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) *
               (de ** 2 - np.cos(e) ** 2 * dlamb ** 2))
    return ddp
event_pdt0.terminal = True


def event_emin(t, x):
    return x[1] - StateLimits.e_min
event_emin.terminal = True


def event_emax(t, x):
    return x[1] - StateLimits.e_max
event_emax.terminal = True


def event_edt0(t, x):
    p, e, lamb, dp, de, dlamb, dp_event, de_event, dlamb_event = x
    if EventParams.model_type == ModelType.EASY:
        dde = (L_2 / J_e) * np.cos(e) + (L_3 / J_e) * np.cos(p) * EventParams.V_s
    elif EventParams.model_type == ModelType.FRICTION:
        dde = (L_2 / J_e) * np.cos(e) + (L_3 / J_e) * np.cos(p) * EventParams.V_s - (mc.d_e / J_e) * de
    elif EventParams.model_type == ModelType.CENTRIPETAL:
        dde = ((L_2 / J_e) * np.cos(e) + (L_3 / J_e) * np.cos(p) * EventParams.V_s - (mc.d_e / J_e) * de -
               np.cos(e) * np.sin(e) * dlamb ** 2)
    return dde
event_edt0.terminal = True

def event_lambmin(t, x):
    return x[2] - StateLimits.lamb_min
event_lambmin.terminal = True


def event_lambmax(t, x):
    return x[2] - StateLimits.lamb_max
event_lambmax.terminal = True


def event_lambdt0(t, x):
    p, e, lamb, dp, de, dlamb, dp_event, de_event, dlamb_event = x
    if EventParams.model_type == ModelType.EASY:
        ddlamb = (L_4 / J_l) * np.cos(e) * np.sin(p) * EventParams.V_s

    if EventParams.model_type == ModelType.FRICTION:
        ddlamb = (L_4 / J_l) * np.cos(e) * np.sin(p) * EventParams.V_s - (mc.d_l / J_l) * dlamb

    if EventParams.model_type == ModelType.CENTRIPETAL:
        ddlamb = (L_4 / J_l) * np.cos(e) * np.sin(p) * EventParams.V_s - (mc.d_l / J_l) * dlamb
    return ddlamb
event_lambdt0.terminal = True


class HeliSimulation(object):
    def __init__(self, theta1_0, theta2_0, theta3_0, timeStep):
        """Initializes the simulation"""
        self.currentState = np.array([theta1_0, theta2_0, theta3_0, 0, 0, 0, 0, 0, 0])
        self.should_check_limits = True
        self.statLim = StateLimits()
        self.timeStep = timeStep
        self.model_type = ModelType.CENTRIPETAL
        self.currentTime = 0


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

        ddp_event = ddp*0
        dde_event = dde *0
        ddlamb_event = ddlamb *0

        if self.statLim.p == LimitType.UPPER_LIMIT or self.statLim.p == LimitType.LOWER_LIMIT:
            dp = 0
            ddp = 0

        if self.statLim.e == LimitType.UPPER_LIMIT or self.statLim.e == LimitType.LOWER_LIMIT:
            de = 0
            dde = 0

        if self.statLim.lamb == LimitType.UPPER_LIMIT or self.statLim.lamb == LimitType.LOWER_LIMIT:
            dlamb = 0
            ddlamb = 0

        return np.array([dp, de, dlamb, ddp, dde, ddlamb, ddp_event, dde_event, ddlamb_event])

    def generateEventList(self):
        event_list = []
        # event dic: eventNr -> eventName
        # eventName can be used in if structure
        event_dic = {}
        m = 0
        if self.statLim.p == LimitType.UPPER_LIMIT or self.statLim.p == LimitType.LOWER_LIMIT:
            event_list.append(event_pdt0)
            event_dic.update({m: "event_pdt0"})
            m += 1
        elif self.statLim.p == LimitType.NO_LIMIT_REACHED:
            event_list.append(event_pmin)
            event_dic.update({m: "event_pmin"})
            m += 1
            event_list.append(event_pmax)
            event_dic.update({m: "event_pmax"})
            m += 1
        if self.statLim.e == LimitType.UPPER_LIMIT or self.statLim.e == LimitType.LOWER_LIMIT:
            event_list.append(event_edt0)
            event_dic.update({m: "event_edt0"})
            m += 1
        elif self.statLim.e == LimitType.NO_LIMIT_REACHED:
            event_list.append(event_emin)
            event_dic.update({m: "event_emin"})
            m += 1
            event_list.append(event_emax)
            event_dic.update({m: "event_emax"})
            m += 1
        if self.statLim.lamb == LimitType.UPPER_LIMIT or self.statLim.lamb == LimitType.LOWER_LIMIT:
            event_list.append(event_lambdt0)
            event_dic.update({m: "event_lambdt0"})
            m += 1
        elif self.statLim.lamb == LimitType.NO_LIMIT_REACHED:
            event_list.append(event_lambmin)
            event_dic.update({m: "event_lambmin"})
            m += 1
            event_list.append(event_lambmax)
            event_dic.update({m: "event_lambmax"})
            m += 1
        return event_list, event_dic

    def switchState(self, event, former_state):
        """Switches the discrete state dependent on which event was triggered.
        :param event: function object, that returned 0 to the solver
        :type event: Callable[[float, np.ndarray], float]"""
        p, e, lamb, dp, de, dlamb, _1, _2, _3 = former_state
        if event.__name__ == "event_pmin":
            p = self.statLim.p_min
            dp = 0
            self.statLim.p = LimitType.LOWER_LIMIT
        if event.__name__ == "event_pmax":
            p = self.statLim.p_max
            dp = 0
            self.statLim.p = LimitType.UPPER_LIMIT
        if event.__name__ == "event_emin":
            e = self.statLim.e_min
            de = 0
            self.statLim.e = LimitType.LOWER_LIMIT
        if event.__name__ == "event_emax":
            e = self.statLim.e_max
            de = 0
            self.statLim.e = LimitType.UPPER_LIMIT
        if event.__name__ == "event_lambmin":
            lamb = self.statLim.lamb_min
            dlamb = 0
            self.statLim.lamb = LimitType.LOWER_LIMIT
        if event.__name__ == "event_lambmax":
            lamb = self.statLim.lamb_max
            dlamb = 0
            self.statLim.lamb = LimitType.UPPER_LIMIT
        if event.__name__ == "event_pdt0":
            self.statLim.p = LimitType.NO_LIMIT_REACHED
        if event.__name__ == "event_edt0":
            self.statLim.e = LimitType.NO_LIMIT_REACHED
        if event.__name__ == "event_lambdt0":
            self.statLim.lamb = LimitType.NO_LIMIT_REACHED
        return [p, e, lamb, dp, de, dlamb, _1, _2, _3]


    def simulateSegment(self, t0, tf, x0, V_s, V_d, event_list):
        """Simulates one segment from t0 to tf and takes care of events that occur in that time interval.
        Returns state vector at tf."""
        if t0 == tf:
            print("[ERROR] to is equal to tf at the beginning of simulateSegment()!")
            return x0

        tt = np.linspace(t0, tf, 2)
        # print("t0 = " + str(t0) + " tf = " + str(tf))
        # print("x0 = " + str(x0))
        # print("event_list = " + str(event_list))
        # print("tt = " + str(tt))
        sol = scipy.integrate.solve_ivp(lambda t, x: self.rhs(t, x, V_s, V_d),
                                        (t0, tf), x0,
                                        method='RK45', t_eval=tt, rtol=1e-6, atol=1e-9,
                                        events=event_list)
        # check if events did occur
        if np.size(sol.t_events) != 0:
            # get information about event
            for idx, v in enumerate(sol.t_events):
                if np.size(v) != 0:
                    event_time = v[0]
                    event_nr = idx
                    triggered_event = event_list[event_nr]
            print(triggered_event.__name__ + " was triggered at " + str(event_time))
            # remove occured event from event list
            event_list.remove(triggered_event)
            # integrate from t0 to eventTime
            state_at_te = self.simulateSegment(t0, event_time, x0, V_s, V_d, event_list)
            # switch discrete state and also set the state vector according to the new state
            # e.g. set velocity in boundaries to zero
            switched_state = self.switchState(triggered_event, state_at_te)
            # integrate from eventTime to tf
            state_at_tf = self.simulateSegment(event_time, tf, switched_state, V_s, V_d, event_list)
        else:
            state_at_tf = sol.y[:, -1]

        return state_at_tf

    def processLimitStateMachine(self, V_s, V_d):
        """This function is called at the beginning of every call of calcStep() for checking if
        the discontinuous second derivative has skipped the 0-value.
        It only switches from limit to non-limit, switching from non-limit to limit is only done by the events.
        :return event_blacklist: events that are not to be cared of in the next integration interval"""
        pdt, edt, lambdt, pddt, eddt, lambddt = self.rhs_old(self.currentTime, self.currentState[0:6], V_s, V_d)
        event_blacklist = []
        if self.statLim.p == LimitType.UPPER_LIMIT:
            if pddt <= 0:
                event_blacklist.append(event_pmax)
                self.statLim.p = LimitType.NO_LIMIT_REACHED
        if self.statLim.p == LimitType.LOWER_LIMIT:
            if pddt >= 0:
                event_blacklist.append(event_pmin)
                self.statLim.p = LimitType.NO_LIMIT_REACHED
        if self.statLim.e == LimitType.UPPER_LIMIT:
            if eddt <= 0:
                event_blacklist.append(event_emax)
                self.statLim.e = LimitType.NO_LIMIT_REACHED
        if self.statLim.e == LimitType.LOWER_LIMIT:
            if eddt >= 0:
                event_blacklist.append(event_emin)
                self.statLim.e = LimitType.NO_LIMIT_REACHED
        if self.statLim.lamb == LimitType.UPPER_LIMIT:
            if lambddt <= 0:
                # print("leave lambmax @ processLimitStateMachine()")
                # print(self.currentState)
                # print(self.rhs_old(self.currentTime, self.currentState[0:6], V_s, V_d))
                event_blacklist.append(event_lambmax)
                self.statLim.lamb = LimitType.NO_LIMIT_REACHED
        if self.statLim.lamb == LimitType.LOWER_LIMIT:
            if lambddt >= 0:
                event_blacklist.append(event_lambmin)
                self.statLim.lamb = LimitType.NO_LIMIT_REACHED
        return event_blacklist



    def calcStep(self, V_f, V_b):
        """Returns the state of the system after the next time step
        V_f: voltage of the propeller right at back (of Fig.7) / first endeffector
        V_b: voltage of the propeller right at front (of Fig. 7) / second endeffector"""
        #start = time.time()
        # print("====> calcStep() t = " + str(self.currentTime))
        V_s = V_f + V_b
        V_d = V_f - V_b
        EventParams.V_s = V_s
        EventParams.V_d = V_d
        EventParams.model_type = self.model_type
        event_blacklist = self.processLimitStateMachine(V_s, V_d)
        event_list, event_dic = self.generateEventList()
        for _ in event_blacklist:
            event_list.remove(_)
        self.currentState = self.simulateSegment(self.currentTime, self.currentTime + self.timeStep,
                                                 self.currentState, V_s, V_d, event_list)
        self.currentTime += self.timeStep
        return self.currentState[0:6]
        # checkLimits is called before integrate because the solver calls the rhs function several times
        # between two discrete steps. if there are state transitions between two steps, the solver
        # will probably react unpredictable
        # if self.should_check_limits and 1 == 2:
        #     self.checkLimits()
        # # integrate one time step
        # # for adjusting the performance the number of in-between-steps can be changed
        # loopCondition = True
        # t0_g = self.currentTime
        # tf_g = self.currentTime + self.timeStep
        # x0_g = self.currentState
        # t0 = t0_g
        # tf = tf_g
        # x0 = x0_g
        # m = 0
        # while loopCondition:
        #     event_list, event_dic = self.generateEventList()
        #     tt = np.linspace(t0, tf, 3)
        #     sol = scipy.integrate.solve_ivp(lambda t, x: self.rhs(t, x, V_s, V_d),
        #                                     (t0, tf), x0,
        #                                     method='RK45', t_eval=tt,  rtol=1e-6, atol=1e-9,
        #                                     events=event_list)
        #     # check if there was an event in order to limit the angle states
        #     # TODO: Simulation Time must always stop at +timeStep !
        #     if np.size(sol.t_events) != 0:
        #         m += 1
        #         # get time of event
        #         for idx, v in enumerate(sol.t_events):
        #             if np.size(v) != 0:
        #                 eventTime = v[0]
        #                 eventNr = idx
        #         # 1. simulate until event time
        #         t0 = sol.t[-1]
        #         tf = eventTime
        #         x0 = sol.y[:, -1]
        #         tt = np.linspace(t0, tf, 2)
        #         # print(eventTime)
        #         # print(sol.t)
        #         # print(sol.y)
        #         sol2 = scipy.integrate.solve_ivp(lambda t, x: self.rhs(t, x, V_s, V_d),
        #                                          (t0, tf), x0,
        #                                          method='RK45', t_eval=tt, rtol=1e-6, atol=1e-9,
        #                                          events=event_list)
        #         # print(sol2)
        #         # there should not be an event in this simulation, but if there is probably it is the same event
        #         # as before or something went terribly wrong (like e.g. pitch and elevation become bounded in
        #         # the same microsecond AND the solver confuses the order of them)
        #         # firstly i am not going to address the second problem, but for debugging purpose it will be printed
        #         for idx, v in enumerate(sol2.t_events):
        #             if np.size(v) != 0:
        #                 eventTime2 = v[0]
        #                 eventNr2 = idx
        #                 eventName2 = event_dic[eventNr2]
        #         if np.size(sol2.t_events) != 0:
        #             print("[DEBUG] Event was triggered in second simulation part! Event = " + eventName2)
        #         # 2. Manage state machine
        #         eventName = event_dic[eventNr]
        #         print(str(eventNr) + " ==> " + eventName)
        #         self.currentState = sol2.y[:, -1]
        #         if eventName == "event_pmin":
        #             print("pmin")
        #             self.currentState[0] = StateLimits.p_min - StateLimits.eps_p
        #             self.statLim.p = LimitType.LOWER_LIMIT
        #         if eventName == "event_pmax":
        #             print("pmax")
        #             self.currentState[0] = StateLimits.p_max + StateLimits.eps_p
        #             self.statLim.p = LimitType.UPPER_LIMIT
        #         if eventName == "event_emin":
        #             print("emin")
        #             self.currentState[1] = StateLimits.e_min - StateLimits.eps_e
        #             self.statLim.e = LimitType.LOWER_LIMIT
        #         if eventName == "event_emax":
        #             print("emax")
        #             self.currentState[1] = StateLimits.e_max + StateLimits.eps_e
        #             self.statLim.e = LimitType.UPPER_LIMIT
        #         if eventName == "event_pdt0":
        #             print("p  no limit")
        #             if self.statLim.p == LimitType.UPPER_LIMIT:
        #                 self.currentState[0] = StateLimits.p_max - StateLimits.eps_p
        #             elif self.statLim.p == LimitType.LOWER_LIMIT:
        #                 self.currentState[0] = StateLimits.p_min + StateLimits.eps_p
        #             self.statLim.p = LimitType.NO_LIMIT_REACHED
        #         if eventName == "event_edt0":
        #             print("e  no limit")
        #             if self.statLim.e == LimitType.UPPER_LIMIT:
        #                 self.currentState[1] = StateLimits.e_max - StateLimits.eps_e
        #             elif self.statLim.e == LimitType.LOWER_LIMIT:
        #                 self.currentState[1] = StateLimits.e_min + StateLimits.eps_e
        #             self.statLim.e = LimitType.NO_LIMIT_REACHED
        #         # 3. Set simulation parameters for next part of interval
        #         t0 = eventTime
        #         tf = tf_g
        #         x0 = self.currentState
        #     else:
        #         # if there were no events, just leave the loop
        #         loopCondition = False
        #
        # if m != 0:
        #     # print("t_g")
        #     # print(tf_g)
        #     # print(sol.t[-1])
        #     pass
        # self.currentState = sol.y[:, -1]
        # self.currentTime = sol.t[-1]

        # end = time.time()
        # print(end - start)
        # return self.currentState[0:6]

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
        # reset state machine
        self.statLim.p = LimitType.NO_LIMIT_REACHED
        self.statLim.e = LimitType.NO_LIMIT_REACHED

    def setModelType(self, modelType):
        self.model_type = modelType