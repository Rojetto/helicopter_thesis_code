import numpy as np
import sympy as sp
import scipy.integrate
import copy
from enum import Enum
import time
from ModelConstants import OriginalConstants as mc
from ModelConstants import ModelType


# locally override basic functions to make code work numerically and symbolically
def sin(x):
    if isinstance(x, sp.Expr):
        return sp.sin(x)
    else:
        return np.sin(x)


def cos(x):
    if isinstance(x, sp.Expr):
        return sp.cos(x)
    else:
        return np.cos(x)


def sqrt(x):
    if isinstance(x, sp.Expr):
        return sp.sqrt(x)
    else:
        return np.sqrt(x)


def Fr(w):
    cond_1 = w <= -2 * mc.q2 / mc.p2
    expr_1 = mc.p2 * w + mc.q2
    cond_2 = w <= 0
    expr_2 = - mc.p2 ** 2 / (4 * mc.q2) * w ** 2
    cond_3 = w <= 2 * mc.q1 / mc.p1
    expr_3 = mc.p1 ** 2 / (4 * mc.q1) * w ** 2
    cond_4 = True
    expr_4 = mc.p1 * w - mc.q1

    if isinstance(w, sp.Expr):
        return sp.functions.elementary.piecewise.Piecewise((expr_1, cond_1), (expr_2, cond_2), (expr_3, cond_3), (expr_4, cond_4))
    else:
        if cond_1:
            return expr_1
        elif cond_2:
            return expr_2
        elif cond_3:
            return expr_3
        elif cond_4:
            return expr_4


def Fr_inverse(F):
    cond_1 = F <= - mc.q2
    expr_1 = (F - mc.q2) / mc.p2
    cond_2 = F < 0
    if isinstance(F, sp.Expr) or F <= 0:  # odd workaround to keep np.sqrt from throwing errors
        expr_2 = - sqrt(-4*mc.q2*F) / mc.p2
    else:
        expr_2 = 0
    cond_3 = F < mc.q1
    if isinstance(F, sp.Expr) or F >= 0:
        expr_3 = sqrt(4*mc.q1*F) / mc.p1
    else:
        expr_3 = 0
    cond_4 = True
    expr_4 = (F + mc.q1) / mc.p1

    if isinstance(F, sp.Expr):
        return sp.functions.elementary.piecewise.Piecewise((expr_1, cond_1), (expr_2, cond_2), (expr_3, cond_3),
                                                           (expr_4, cond_4))
    else:
        if cond_1:
            return expr_1
        elif cond_2:
            return expr_2
        elif cond_3:
            return expr_3
        elif cond_4:
            return expr_4


class LimitType(Enum):
    NO_LIMIT_REACHED = 0
    UPPER_LIMIT = 1
    LOWER_LIMIT = -1


class StateLimits(object):
    """
    Helper class just for saving which states have reached their limit at the current time step,
    but also for saving some attributes about the limits
    """
    p_max = 100 / 180.0 * np.pi
    p_min = -p_max
    e_max = 29 / 180.0 * np.pi
    e_min = -e_max
    lamb_max = 180 / 180.0 * np.pi
    lamb_min = -lamb_max
    # these epsillon values are necessary for set_current_state_and_time_and_check_limits
    eps_p = 1 / 180.0 * np.pi
    eps_e = 1 / 180.0 * np.pi
    eps_lamb = 1 / 180.0 * np.pi

    def __init__(self):
        self.p = LimitType.NO_LIMIT_REACHED
        self.e = LimitType.NO_LIMIT_REACHED
        self.lamb = LimitType.NO_LIMIT_REACHED


class EventParams(object):
    V_s = -1
    V_d = -1
    model_type = None
    current_disturbance = [0, 0, 0, 0, 0]
    dynamic_inertia_torque = True


def system_f(x, u, model_type: ModelType, dynamic_inertia):
    term_friction = True if model_type >= ModelType.FRICTION else False
    term_centripetal = True if model_type >= ModelType.CENTRIPETAL else False
    term_motor_pt1 = True if model_type >= ModelType.ROTORSPEED else False
    term_motor_reaction = True if model_type >= ModelType.ROTORSPEED else False
    #term_motor_nonlinear = True if model_type >= ModelType.ROTORSPEED else False
    term_motor_nonlinear = True
    term_rotor_gyro = True if model_type >= ModelType.GYROMOMENT else False
    term_dynamic_inertia = dynamic_inertia

    g = 9.81

    phi, eps, lamb, dphi, deps, dlamb, wf, wb = x
    uf, ub = u

    m_h, m_c, l_h, l_c, l_p, d_h, d_c = mc.m_h, mc.m_c, mc.l_h, mc.l_c, mc.l_p, mc.d_h, mc.d_c

    if term_dynamic_inertia:
        p_phi_1 = m_h*(l_p**2+d_h**2)
        p_eps_1 = m_c*(l_c**2+d_c**2) + m_h*(l_h**2+d_h**2) + m_h*sin(phi)**2*(l_p**2-d_h**2)
        p_lamb_1 = -d_c**2*m_c*cos(eps)**2-d_c*l_c*m_c*sin(2*eps)-d_h**2*m_h*cos(eps)**2*cos(phi)**2+d_h**2*m_h - d_h*l_h*m_h/2*(sin(2*eps-phi)+sin(2*eps+phi))+l_c**2*m_c*cos(eps)**2+l_h**2*m_h*cos(eps)**2+l_p**2*m_h*cos(eps)**2*cos(phi)**2-l_p**2*m_h*cos(eps)**2+l_p**2*m_h
    else:
        p_phi_1 = m_h*(l_p**2+d_h**2)
        p_eps_1 = m_c*(l_c**2+d_c**2) + m_h*(l_h**2+d_h**2)
        p_lamb_1 = m_h*(l_h**2+l_p**2) + m_c*l_c**2

    p_phi_2 = - g*d_h*m_h*cos(eps)
    p_eps_2 = g*(d_c*m_c - d_h*m_h*cos(phi))
    p_eps_3 = g*(l_h*m_h - m_c*l_c)

    if term_motor_pt1:
        dwf = 1 / mc.T_w * (mc.K_w * uf - wf)
        dwb = 1 / mc.T_w * (mc.K_w * ub - wb)
    else:
        wf = uf
        wb = ub
        dwf = 0
        dwb = 0

    if term_motor_nonlinear:
        Ff = Fr(wf)
        Fb = Fr(wb)
    else:
        Ff = wf
        Fb = wb

    Fs = Ff + Fb
    Fd = Ff - Fb

    ws = wf + wb
    wd = wf - wb

    ddphi_rhs = -p_phi_2 * sin(phi) + Fd * l_p
    ddeps_rhs = -p_eps_2 * sin(eps) - p_eps_3 * cos(eps) + Fs * l_h * cos(phi)
    ddlamb_rhs = Fs * l_h * cos(eps) * sin(phi) - Fd * l_p * sin(eps)

    if term_centripetal:
        #ddphi_rhs = ddphi_rhs + p_phi_1 * cos(phi) * sin(phi) * (deps ** 2 - cos(eps) ** 2 * dlamb ** 2)
        #ddeps_rhs = ddeps_rhs - J_eps_2 * cos(eps) * sin(eps) * dlamb ** 2
        pass

    if term_friction:
        ddphi_rhs = ddphi_rhs - mc.mu_phi * dphi
        ddeps_rhs = ddeps_rhs - mc.mu_eps * deps
        ddlamb_rhs = ddlamb_rhs - mc.mu_lamb * dlamb

    if term_motor_reaction:
        #ddeps_rhs = ddeps_rhs + sin(phi) * mc.K_m * wd
        #ddlamb_rhs = ddlamb_rhs - cos(eps) * cos(phi) * mc.K_m * wd
        pass

    if term_rotor_gyro:
        #ddphi_rhs = ddphi_rhs - mc.J_m * cos(phi) * deps * wd + mc.J_m * sin(phi) * cos(eps) * dlamb * wd
        #ddeps_rhs = ddeps_rhs + mc.J_m * cos(phi) * dphi * wd - mc.J_m * cos(phi) * sin(eps) * dlamb * wd
        #ddlamb_rhs = ddlamb_rhs + mc.J_m * sin(phi) * cos(eps) * dphi * wd
        pass

    ddphi = ddphi_rhs / p_phi_1
    ddeps = ddeps_rhs / p_eps_1
    ddlamb = ddlamb_rhs / p_lamb_1

    if isinstance(dphi, sp.Expr):
        dx = sp.Matrix([dphi, deps, dlamb, ddphi, ddeps, ddlamb, dwf, dwb])
    else:
        dx = np.array([dphi, deps, dlamb, ddphi, ddeps, ddlamb, dwf, dwb])

    return dx


def event_pmin(t, x):
    return x[0] - StateLimits.p_min
event_pmin.terminal = True


def event_pmax(t, x):
    return x[0] - StateLimits.p_max
event_pmax.terminal = True


def event_pdt0(t, x):
    """Checks if the second derivative has crossed zero."""
    z_phi, z_eps, z_lamb, z_wf, z_wb = EventParams.current_disturbance
    J_phi, J_eps1, J_lamb = getInertia(x, EventParams.dynamic_inertia_torque)

    u = np.array([(EventParams.V_s + EventParams.V_d) / 2, (EventParams.V_s - EventParams.V_d) / 2])
    dx = system_f(x, u, EventParams.model_type, EventParams.dynamic_inertia_torque)
    ddphi = dx[3]
    # Apply disturbance
    ddphi += z_phi / J_phi

    return ddphi
event_pdt0.terminal = True


def event_emin(t, x):
    return x[1] - StateLimits.e_min
event_emin.terminal = True


def event_emax(t, x):
    return x[1] - StateLimits.e_max
event_emax.terminal = True


def event_edt0(t, x):
    """Checks if the second derivative has crossed zero."""
    z_phi, z_eps, z_lamb, z_wf, z_wb = EventParams.current_disturbance
    J_phi, J_eps1, J_lamb = getInertia(x, EventParams.dynamic_inertia_torque)

    u = np.array([(EventParams.V_s + EventParams.V_d) / 2, (EventParams.V_s - EventParams.V_d) / 2])
    dx = system_f(x, u, EventParams.model_type, EventParams.dynamic_inertia_torque)
    ddeps = dx[4]
    # Apply disturbance
    ddeps += z_eps / J_eps1

    return ddeps
event_edt0.terminal = True

def event_lambmin(t, x):
    return x[2] - StateLimits.lamb_min
event_lambmin.terminal = True


def event_lambmax(t, x):
    return x[2] - StateLimits.lamb_max
event_lambmax.terminal = True


def event_lambdt0(t, x):
    """Checks if the second derivative has crossed zero."""
    z_phi, z_eps, z_lamb, z_wf, z_wb = EventParams.current_disturbance
    J_phi, J_eps1, J_lamb = getInertia(x, EventParams.dynamic_inertia_torque)

    u = np.array([(EventParams.V_s + EventParams.V_d) / 2, (EventParams.V_s - EventParams.V_d) / 2])
    dx = system_f(x, u, EventParams.model_type, EventParams.dynamic_inertia_torque)
    ddlamb = dx[5]
    # Apply disturbance
    ddlamb += z_lamb / J_lamb

    return ddlamb
event_lambdt0.terminal = True


def getInertia(x, dynamic_inertia_torque):
    """Computes inertia torque dependend on """
    phi, eps, lamb, dphi, deps, dlamb, wf, wb = x

    m_h, m_c, l_h, l_c, l_p, d_h, d_c = mc.m_h, mc.m_c, mc.l_h, mc.l_c, mc.l_p, mc.d_h, mc.d_c

    if dynamic_inertia_torque:
        J_phi = m_h * (l_p ** 2 + d_h ** 2)
        J_eps = m_c * (l_c ** 2 + d_c ** 2) + m_h * (l_h ** 2 + d_h ** 2) + m_h * sin(phi) ** 2 * (
                    l_p ** 2 - d_h ** 2)
        J_lamb = -d_c ** 2 * m_c * cos(eps) ** 2 - d_c * l_c * m_c * sin(2 * eps) - d_h ** 2 * m_h * cos(
            eps) ** 2 * cos(phi) ** 2 + d_h ** 2 * m_h - d_h * l_h * m_h / 2 * (
                               sin(2 * eps - phi) + sin(2 * eps + phi)) + l_c ** 2 * m_c * cos(
            eps) ** 2 + l_h ** 2 * m_h * cos(eps) ** 2 + l_p ** 2 * m_h * cos(eps) ** 2 * cos(
            phi) ** 2 - l_p ** 2 * m_h * cos(eps) ** 2 + l_p ** 2 * m_h
    else:
        J_phi = m_h * (l_p ** 2 + d_h ** 2)
        J_eps = m_c * (l_c ** 2 + d_c ** 2) + m_h * (l_h ** 2 + d_h ** 2)
        J_lamb = m_h * (l_h ** 2 + l_p ** 2) + m_c * l_c ** 2

    return np.array([J_phi, J_eps, J_lamb])


class HeliSimulation(object):
    def __init__(self, theta1_0, theta2_0, theta3_0, time_step):
        """Initializes the simulation"""
        self.currentState = np.array([theta1_0, theta2_0, theta3_0, 0, 0, 0, 0, 0])
        self.should_check_limits = True
        self.dynamic_inertia_torque = True
        self.statLim = StateLimits()
        self.timeStep = time_step
        self.model_type = ModelType.CENTRIPETAL
        self.currentTime = 0

    def rhs_no_limits(self, t, x, v_s, v_d, current_disturbance):
        """Right hand side of the differential equation being integrated. This function does simply calculate the
        state derivative, not dependend on the discrete limit state."""
        z_phi, z_eps, z_lamb, z_wf, z_wb = current_disturbance

        J_phi, J_eps1, J_lamb = getInertia(x, self.dynamic_inertia_torque)

        u = np.array([(v_s+v_d)/2, (v_s-v_d)/2])
        dx = system_f(x, u, self.model_type, self.dynamic_inertia_torque)

        # Apply disturbance
        dx[3] += z_phi / J_phi
        dx[4] += z_eps / J_eps1
        dx[5] += z_lamb / J_lamb
        dx[6] += z_wf
        dx[7] += z_wb

        return dx

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
            print("[switch_state] leaving e limit")
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
                print("[process_limit_state_machine] Leaving lower limit of e")
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
        # J_p, J_e, J_l = getInertia(self.currentState, self.dynamic_inertia_torque)
        # print("S-rhs_no_limits: J_p = " + str(J_p) + ", J_e = " + str(J_e) + ", J_l = " + str(J_l))
        # start = time.time()
        # print("====> calcStep() t = " + str(self.currentTime))
        v_s = v_f + v_b
        v_d = v_f - v_b
        EventParams.V_s = v_s
        EventParams.V_d = v_d
        EventParams.model_type = self.model_type
        EventParams.current_disturbance = current_disturbance
        EventParams.dynamic_inertia_torque = self.dynamic_inertia_torque
        # ###PATCH
        if self.model_type == ModelType.ROTORSPEED or self.model_type == ModelType.GYROMOMENT:
            # if we are simulating the models with f and b, we do NOT need to consider that the second derivative
            # of p,e,lambda is discontinuous
            event_blacklist = []
            # save state and signs of second derivative in order to find malfunctions of the event system later
            statlim_before = copy.copy(self.statLim)
            _1, _2, _3, pddt_before, eddt_before, lambddt_before, _4, _5 = self.rhs_no_limits(self.currentTime,
                                                                                              self.currentState, v_s, v_d,
                                                                                              current_disturbance)
            if self.statLim.e == LimitType.LOWER_LIMIT and eddt_before > 0:
                print("something went horribly wrong with e")
        else:
            # if we are simulating the models without f and b, we need to consider that the second derivative
            # of p,e,lambda is discontinuous
            event_blacklist = self.process_limit_state_machine(v_s, v_d, current_disturbance)
        # ### PATCH END
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

        # ###PATCH
        if self.model_type == ModelType.ROTORSPEED or self.model_type == ModelType.GYROMOMENT:
            # first problem: qddt goes around 0 and confuses the zero crossing detection
            # ===> detect and reset

            # we can set v_s and v_d to zero because they have no influence on the second derivatives
            # in these model types
            _1, _2, _3, pddt_after, eddt_after, lambddt_after, _4, _5 = self.rhs_no_limits(self.currentTime,
                                                                                              self.currentState, 0,
                                                                                              0,
                                                                                              current_disturbance)
            # lets check if limits were left although signs of *ddt didnt change
            if self.statLim.p == LimitType.NO_LIMIT_REACHED and statlim_before.p != LimitType.NO_LIMIT_REACHED:
                if np.sign(pddt_before) == np.sign(pddt_after):
                    # there was a malfunction in the event system. reset this coordinate
                    self.statLim.p = statlim_before.p
                    dp = 0
                    if statlim_before.p == LimitType.UPPER_LIMIT:
                        p = self.statLim.p_max
                    elif statlim_before.p == LimitType.LOWER_LIMIT:
                        p = self.statLim.p_min
                    # save it to the state
                    self.currentState[0] = p
                    self.currentState[3] = dp
            if self.statLim.e == LimitType.NO_LIMIT_REACHED and statlim_before.e != LimitType.NO_LIMIT_REACHED:
                if np.sign(eddt_before) == np.sign(eddt_after):
                    # there was a malfunction in the event system. reset this coordinate
                    print("malfunction in e limit detection")
                    self.statLim.e = statlim_before.e
                    de = 0
                    if statlim_before.e == LimitType.UPPER_LIMIT:
                        e = self.statLim.e_max
                    elif statlim_before.e == LimitType.LOWER_LIMIT:
                        e = self.statLim.e_min
                    # save it to the state
                    self.currentState[1] = e
                    self.currentState[4] = de
            if self.statLim.lamb == LimitType.NO_LIMIT_REACHED and statlim_before.lamb != LimitType.NO_LIMIT_REACHED:
                if np.sign(lambddt_before) == np.sign(lambddt_after):
                    # there was a malfunction in the event system. reset this coordinate
                    self.statLim.lamb = statlim_before.lamb
                    dlamb = 0
                    if statlim_before.lamb == LimitType.UPPER_LIMIT:
                        lamb = self.statLim.lamb_max
                    elif statlim_before.lamb == LimitType.LOWER_LIMIT:
                        lamb = self.statLim.lamb_min
                    # save it to the state
                    self.currentState[2] = lamb
                    self.currentState[5] = dlamb

            # second problem: a limit was set although qddt drags in the different direction
            if self.statLim.p != LimitType.NO_LIMIT_REACHED and statlim_before.p == LimitType.NO_LIMIT_REACHED:
                if self.statLim.p == LimitType.UPPER_LIMIT and pddt_after < 0:
                    print("detected qddt drags away from the limit")
                    self.statLim.p = LimitType.NO_LIMIT_REACHED
                if self.statLim.p == LimitType.LOWER_LIMIT and pddt_after > 0:
                    print("detected qddt drags away from the limit")
                    self.statLim.p = LimitType.NO_LIMIT_REACHED
            if self.statLim.e != LimitType.NO_LIMIT_REACHED and statlim_before.e == LimitType.NO_LIMIT_REACHED:
                if self.statLim.e == LimitType.UPPER_LIMIT and eddt_after < 0:
                    print("detected qddt drags away from the limit")
                    self.statLim.e = LimitType.NO_LIMIT_REACHED
                if self.statLim.e == LimitType.LOWER_LIMIT and eddt_after > 0:
                    print("detected qddt drags away from the limit")
                    self.statLim.e = LimitType.NO_LIMIT_REACHED
            if self.statLim.lamb != LimitType.NO_LIMIT_REACHED and statlim_before.lamb == LimitType.NO_LIMIT_REACHED:
                if self.statLim.lamb == LimitType.UPPER_LIMIT and lambddt_after < 0:
                    print("detected qddt drags away from the limit")
                    self.statLim.lamb = LimitType.NO_LIMIT_REACHED
                if self.statLim.lamb == LimitType.LOWER_LIMIT and lambddt_after > 0:
                    print("detected qddt drags away from the limit")
                    self.statLim.lamb = LimitType.NO_LIMIT_REACHED
        # ### PATCH END
        return self.currentState

    def get_current_state(self):
        return self.currentState

    def get_current_time(self):
        return self.currentTime

    def set_current_state_and_time(self, state, sim_time=0.0):
        """state = p, e, lamb, dp, de, dlamb, f_speed, b_speed = x"""
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

    def set_dynamic_inertia_torque(self, dynamic_inertia_torque):
        self.dynamic_inertia_torque = dynamic_inertia_torque
        # print("Dynamic intertia torque: " + str(self.dynamic_inertia_torque))

