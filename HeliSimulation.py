import numpy as np
import scipy.integrate
import ModelConstants as mc
from enum import Enum
import time
from ModelConstants import ModelType


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
    current_disturbance = [0, 0, 0, 0, 0]


def event_pmin(t, x):
    return x[0] - StateLimits.p_min
event_pmin.terminal = True


def event_pmax(t, x):
    return x[0] - StateLimits.p_max
event_pmax.terminal = True


def event_pdt0(t, x):
    """Checks if the second derivative has crossed zero."""
    p, e, lamb, dp, de, dlamb, f_speed, b_speed = x
    z_p, z_e, z_lamb, z_f, z_b = EventParams.current_disturbance
    J_p, J_e, J_l = getInertia(x, EventParams.model_type)

    if EventParams.model_type == ModelType.EASY:
        ddp = (L_1 / J_p) * EventParams.V_d
    elif EventParams.model_type == ModelType.FRICTION:
        ddp = (L_1 / J_p) * EventParams.V_d - (mc.d_p / J_p) * dp
    elif EventParams.model_type == ModelType.CENTRIPETAL:
        ddp = ((L_1 / J_p) * EventParams.V_d - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) *
               (de ** 2 - np.cos(e) ** 2 * dlamb ** 2))
    elif EventParams.model_type == ModelType.ROTORSPEED:
        ddp = (L_1 * mc.K / J_p) * (f_speed - b_speed) - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (
                    de ** 2 - np.cos(e) ** 2 * dlamb ** 2)

    # Apply disturbance
    ddp += z_p / J_p

    return ddp
event_pdt0.terminal = True


def event_emin(t, x):
    return x[1] - StateLimits.e_min
event_emin.terminal = True


def event_emax(t, x):
    return x[1] - StateLimits.e_max
event_emax.terminal = True


def event_edt0(t, x):
    p, e, lamb, dp, de, dlamb, f_speed, b_speed = x
    z_p, z_e, z_lamb, z_f, z_b = EventParams.current_disturbance
    J_p, J_e, J_l = getInertia(x, EventParams.model_type)

    if EventParams.model_type == ModelType.EASY:
        dde = (L_2 / J_e) * np.cos(e) + (L_3 / J_e) * np.cos(p) * EventParams.V_s
    elif EventParams.model_type == ModelType.FRICTION:
        dde = (L_2 / J_e) * np.cos(e) + (L_3 / J_e) * np.cos(p) * EventParams.V_s - (mc.d_e / J_e) * de
    elif EventParams.model_type == ModelType.CENTRIPETAL:
        dde = ((L_2 / J_e) * np.cos(e) + (L_3 / J_e) * np.cos(p) * EventParams.V_s - (mc.d_e / J_e) * de -
               np.cos(e) * np.sin(e) * dlamb ** 2)
    elif EventParams.model_type == ModelType.ROTORSPEED:
        dde = (L_2 / J_e) * np.cos(e) + (L_3 * mc.K / J_e) * np.cos(p) * (f_speed + b_speed) - (
                    mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dlamb ** 2 + np.sin(p) * mc.K_m * (f_speed-b_speed)

    # Apply disturbance
    dde += z_e / J_e

    return dde
event_edt0.terminal = True

def event_lambmin(t, x):
    return x[2] - StateLimits.lamb_min
event_lambmin.terminal = True


def event_lambmax(t, x):
    return x[2] - StateLimits.lamb_max
event_lambmax.terminal = True


def event_lambdt0(t, x):
    p, e, lamb, dp, de, dlamb, f_speed, b_speed = x
    z_p, z_e, z_lamb, z_f, z_b = EventParams.current_disturbance
    J_p, J_e, J_l = getInertia(x, EventParams.model_type)

    if EventParams.model_type == ModelType.EASY:
        ddlamb = (L_4 / J_l) * np.cos(e) * np.sin(p) * EventParams.V_s
    elif EventParams.model_type == ModelType.FRICTION:
        ddlamb = (L_4 / J_l) * np.cos(e) * np.sin(p) * EventParams.V_s - (mc.d_l / J_l) * dlamb
    elif EventParams.model_type == ModelType.CENTRIPETAL:
        ddlamb = (L_4 / J_l) * np.cos(e) * np.sin(p) * EventParams.V_s - (mc.d_l / J_l) * dlamb
    elif EventParams.model_type == ModelType.ROTORSPEED:
        ddlamb = (L_4 * mc.K / J_l) * np.cos(e) * np.sin(p) * (f_speed + b_speed) - (mc.d_l / J_l) * dlamb + np.cos(e) * np.cos(p) * mc.K_m * (b_speed-f_speed)

    # Apply disturbance
    ddlamb += z_lamb / J_l

    return ddlamb
event_lambdt0.terminal = True

def getInertia(x,model):
    p, e, lamb, dp, de, dlamb, f_speed, b_speed = x

    if (model == ModelType.EASY) or (model == ModelType.FRICTION):
        J_p = 2 * mc.m_p * mc.l_p ** 2
        J_e = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * mc.l_h ** 2
        J_l = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p ** 2)
    else:
        J_p = 2 * mc.m_p * mc.l_p ** 2
        J_e = mc.m_c * mc.l_c ** 2 + 2 * mc.m_p * (mc.l_h ** 2 + mc.l_p **2 * np.sin(p)**2 )
        J_l = mc.m_c * mc.l_c ** 2 * np.cos(e)**2 + 2 * mc.m_p * ((mc.l_h* np.cos(e))**2 + (mc.l_p * np.sin(p) * np.cos(e)) ** 2 + (mc.l_p * np.cos(p))**2)

    return np.array([J_p, J_e, J_l])


class HeliSimulation(object):
    def __init__(self, theta1_0, theta2_0, theta3_0, time_step):
        """Initializes the simulation"""
        self.currentState = np.array([theta1_0, theta2_0, theta3_0, 0, 0, 0, 0, 0])
        self.should_check_limits = True
        self.statLim = StateLimits()
        self.timeStep = time_step
        self.model_type = ModelType.CENTRIPETAL
        self.currentTime = 0

    def rhs_no_limits(self, t, x, v_s, v_d, current_disturbance):
        """Right hand side of the differential equation being integrated. This function does simply calculate the
        state derivative, not dependend on the discrete limit state."""
        p, e, lamb, dp, de, dlamb, f_speed, b_speed = x
        z_p, z_e, z_lamb, z_f, z_b = current_disturbance

        J_p, J_e, J_l = getInertia(x, self.model_type)

        v_f = (v_s + v_d) / 2
        v_b = (v_s - v_d) / 2

        # calculate dp, de and dlamb
        if self.model_type == ModelType.EASY:
            ddp = (L_1 / J_p) * v_d
            dde = (L_2/J_e) * np.cos(e) + (L_3/J_e) * np.cos(p) * v_s
            ddlamb = (L_4/J_l) * np.cos(e) * np.sin(p) * v_s
            df_speed, db_speed = 0, 0

        if self.model_type == ModelType.FRICTION:
            ddp = (L_1 / J_p) * v_d - (mc.d_p / J_p) * dp
            dde = (L_2/J_e) * np.cos(e) + (L_3/J_e) * np.cos(p) * v_s - (mc.d_e / J_e) * de
            ddlamb = (L_4/J_l) * np.cos(e) * np.sin(p) * v_s - (mc.d_l / J_l) * dlamb
            df_speed, db_speed = 0, 0

        if self.model_type == ModelType.CENTRIPETAL:
            ddp = (L_1 / J_p) * v_d - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dlamb ** 2)
            dde = (L_2/J_e) * np.cos(e) + (L_3/J_e) * np.cos(p) * v_s - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dlamb ** 2
            ddlamb = (L_4/J_l) * np.cos(e) * np.sin(p) * v_s - (mc.d_l / J_l) * dlamb
            df_speed, db_speed = 0, 0

        if self.model_type == ModelType.ROTORSPEED:
            df_speed = - f_speed / mc.T_f + mc.K_f / mc.T_f * v_f
            db_speed = - b_speed / mc.T_b + mc.K_b/mc.T_b * v_b
            ddp = (L_1*mc.K/J_p) * (f_speed - b_speed) - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dlamb ** 2)
            dde = (L_2/J_e) * np.cos(e) + (L_3*mc.K/J_e) * np.cos(p) * (f_speed + b_speed) - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dlamb ** 2 + np.sin(p) * mc.K_m * (f_speed-b_speed)
            ddlamb = (L_4*mc.K/J_l) * np.cos(e) * np.sin(p) * (f_speed + b_speed) - (mc.d_l / J_l) * dlamb + np.cos(e) * np.cos(p) * mc.K_m * (b_speed-f_speed)

        if self.model_type == ModelType.GYROMOMENT:
            df_speed = - f_speed / mc.T_f + mc.K_f / mc.T_f * v_f
            db_speed = - b_speed / mc.T_b + mc.K_b/mc.T_b * v_b
            ddp = (L_1*mc.K/J_p) * (f_speed - b_speed) - (mc.d_p / J_p) * dp + np.cos(p) * np.sin(p) * (de ** 2 - np.cos(e) ** 2 * dlamb ** 2) \
                  + np.cos(p) * de * mc.J_m *(b_speed - f_speed) + np.sin(p) * np.cos(e) * mc.J_m * (f_speed - b_speed)
            dde = (L_2/J_e) * np.cos(e) + (L_3*mc.K/J_e) * np.cos(p) * (f_speed + b_speed) - (mc.d_e / J_e) * de - np.cos(e) * np.sin(e) * dlamb ** 2 + np.sin(p) * mc.K_m * (f_speed-b_speed) \
                  + np.cos(p) * dp * mc.J_m * (f_speed -b_speed) + np.sin(e) * np.cos(p) * dlamb * mc.J_m *(b_speed - f_speed)
            ddlamb = (L_4*mc.K/J_l) * np.cos(e) * np.sin(p) * (f_speed + b_speed) - (mc.d_l / J_l) * dlamb + np.cos(e) * np.cos(p) * mc.K_m * (b_speed-f_speed)\
                  + np.sin(p) * np.cos(e) * dp * mc.J_m *(f_speed - b_speed) + np.sin(p) * np.cos(e) * dlamb * mc.J_m *(f_speed - b_speed)


        # Apply disturbance
        ddp += z_p / J_p
        dde += z_e / J_e
        ddlamb += z_lamb / J_l
        # ToDo: make sure that df and db are handled correctly in these formulas
        df_speed += z_f
        db_speed += z_b

        return np.array([dp, de, dlamb, ddp, dde, ddlamb, df_speed, db_speed])

    def rhs(self, t, x, v_s, v_d, current_disturbance):
        """Right hand side of the differential equation being integrated. Behaves according to the
        state limit machine."""
        p, e, lamb, dp, de, dlamb, f_speed, b_speed = x

        dp, de, dlamb, ddp, dde, ddlamb, df_speed, db_speed = self.rhs_no_limits(t, x, v_s, v_d, current_disturbance)

        if self.statLim.p == LimitType.UPPER_LIMIT or self.statLim.p == LimitType.LOWER_LIMIT:
            dp = 0
            ddp = 0

        if self.statLim.e == LimitType.UPPER_LIMIT or self.statLim.e == LimitType.LOWER_LIMIT:
            de = 0
            dde = 0

        if self.statLim.lamb == LimitType.UPPER_LIMIT or self.statLim.lamb == LimitType.LOWER_LIMIT:
            dlamb = 0
            ddlamb = 0

        return np.array([dp, de, dlamb, ddp, dde, ddlamb, df_speed, db_speed])


    def generate_event_list(self):
        """Generates the event_list for the next integration interval depening on the limitation state.
        :return event_list: list of function objects for solve_ivp"""
        event_list = []
        # eventName can be used in if structure
        if self.statLim.p == LimitType.UPPER_LIMIT or self.statLim.p == LimitType.LOWER_LIMIT:
            event_list.append(event_pdt0)
        elif self.statLim.p == LimitType.NO_LIMIT_REACHED:
            event_list.append(event_pmin)
            event_list.append(event_pmax)
        if self.statLim.e == LimitType.UPPER_LIMIT or self.statLim.e == LimitType.LOWER_LIMIT:
            event_list.append(event_edt0)
        elif self.statLim.e == LimitType.NO_LIMIT_REACHED:
            event_list.append(event_emin)
            event_list.append(event_emax)
        if self.statLim.lamb == LimitType.UPPER_LIMIT or self.statLim.lamb == LimitType.LOWER_LIMIT:
            event_list.append(event_lambdt0)
        elif self.statLim.lamb == LimitType.NO_LIMIT_REACHED:
            event_list.append(event_lambmin)
            event_list.append(event_lambmax)
        return event_list

    def switch_state(self, event, state):
        """Switches the discrete state dependent on which event was triggered.
        :param event: function object, that returned 0 to the solver
        :param state: current state, needed in order to alter the state
        :type event: Callable[[float, np.ndarray], float]"""
        p, e, lamb, dp, de, dlamb, f_speed, b_speed = state
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
        return [p, e, lamb, dp, de, dlamb, f_speed, b_speed]

    def simulate_segment(self, t0, tf, x0, v_s, v_d, current_disturbance, event_list):
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
        sol = scipy.integrate.solve_ivp(lambda t, x: self.rhs(t, x, v_s, v_d, current_disturbance),
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
            state_at_te = self.simulate_segment(t0, event_time, x0, v_s, v_d, current_disturbance, event_list)
            # switch discrete state and also set the state vector according to the new state
            # e.g. set velocity in boundaries to zero
            switched_state = self.switch_state(triggered_event, state_at_te)
            # integrate from eventTime to tf
            state_at_tf = self.simulate_segment(event_time, tf, switched_state, v_s, v_d, current_disturbance,
                                                event_list)
        else:
            state_at_tf = sol.y[:, -1]

        return state_at_tf

    def process_limit_state_machine(self, V_s, V_d, current_disturbance):
        """This function is called at the beginning of every call of calcStep() for checking if
        the discontinuous second derivative has skipped the 0-value.
        It only switches from limit to non-limit, switching from non-limit to limit is only done by the events.
        :return event_blacklist: events that are not to be cared of in the next integration interval"""
        pdt, edt, lambdt, pddt, eddt, lambddt, df_speed, db_speed = self.rhs_no_limits(self.currentTime,
                                                                                       self.currentState, V_s, V_d,
                                                                                       current_disturbance)
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
                # print(self.rhs_no_limits(self.currentTime, self.currentState[0:6], V_s, V_d))
                event_blacklist.append(event_lambmax)
                self.statLim.lamb = LimitType.NO_LIMIT_REACHED
        if self.statLim.lamb == LimitType.LOWER_LIMIT:
            if lambddt >= 0:
                event_blacklist.append(event_lambmin)
                self.statLim.lamb = LimitType.NO_LIMIT_REACHED
        return event_blacklist


    def calc_step(self, v_f, v_b, current_disturbance):
        """Returns the state of the system after the next time step
        :param v_f: voltage of the propeller right at back (of Fig.7) / first endeffector
        :param v_b: voltage of the propeller right at front (of Fig. 7) / second endeffector
        :param current_disturbance: np-array with current disturbance for p, e, lambda, f and b
        [0] ==> p, [1] ==> e, [2] ==> lambda, [3] ==> f, [4] ==> b
        """
        # start = time.time()
        # print("====> calcStep() t = " + str(self.currentTime))
        v_s = v_f + v_b
        v_d = v_f - v_b
        EventParams.V_s = v_s
        EventParams.V_d = v_d
        EventParams.model_type = self.model_type
        EventParams.current_disturbance = current_disturbance
        event_blacklist = self.process_limit_state_machine(v_s, v_d, current_disturbance)
        # if the event_list is empty, no events can be triggered that limit the angles
        # because process_limit_state_machine() does not limit anything it can still be called
        if self.should_check_limits:
            event_list = self.generate_event_list()
        else:
            event_list = []
        for _ in event_blacklist:
            event_list.remove(_)
        self.currentState = self.simulate_segment(self.currentTime, self.currentTime + self.timeStep,
                                                  self.currentState, v_s, v_d, current_disturbance, event_list)
        self.currentTime += self.timeStep
        return self.currentState

    def get_current_state(self):
        return self.currentState

    def get_current_time(self):
        return self.currentTime

    def set_current_state_and_time(self, state, sim_time=0.0):
        self.currentTime = sim_time
        self.currentState = np.array(state)
        # reset state machine
        self.statLim.p = LimitType.NO_LIMIT_REACHED
        self.statLim.e = LimitType.NO_LIMIT_REACHED
        self.statLim.lamb = LimitType.NO_LIMIT_REACHED

    def set_model_type(self, modelType):
        self.model_type = modelType

    def get_model_type(self):
        return self.model_type

    def set_should_limit(self, should_check_limits):
        self.should_check_limits = should_check_limits

