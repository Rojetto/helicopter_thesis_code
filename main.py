import matplotlib
matplotlib.use('Qt5Agg')
import os

import logger
import zentripedal_validation as val
from JoystickWidget import JoystickWidget
from helicontrollers.util import FeedForwardMethod, compute_feed_forward_static, compute_feed_forward_flatness
from helicontrollers.ManualController import ManualController
from helicontrollers.CascadePidController import CascadePidController
from helicontrollers.MCascadePidController import MCascadePidController
from helicontrollers.LqrController import LqrController
from helicontrollers.QuasistaticFlatnessController import QuasistaticFlatnessController
from helicontrollers.FeedbackLinearizationController import FeedbackLinearizationController
from helicontrollers.InteractiveController import InteractiveController

from ControllerFrame import ControllerFrame
from ModelFrame import ModelFrame
from TrajectoryFrame import TrajectoryFrame
from DisturbanceFrame import DisturbanceFrame
from FeedforwardFrame import FeedforwardFrame
from ObserverFrame import ObserverFrame
from helicontrollers.DirectPidController import DirectPidController
from helicontrollers.PolePlacementController import PolePlacementController
from helicontrollers.TimeVariantController import TimeVariantController

from HelicopterModel import HelicopterModel
from HelicopterModelEstimated import HelicopterModelEstimated
from HeliSimulation import HeliSimulation
from MyQVTKRenderWindowInteractor import QVTKRenderWindowInteractor

import sys
import vtk
from PyQt5 import QtCore, QtWidgets
import numpy as np


class mainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setWindowTitle("Helicopter Simulation")

        # GUI setup
        frame = QtWidgets.QFrame()
        main_layout = QtWidgets.QVBoxLayout()
        frame.setLayout(main_layout)
        self.setCentralWidget(frame)

        # VTK setup
        vtk_widget = QVTKRenderWindowInteractor(frame)
        vtk_renderer = vtk.vtkRenderer()
        vtk_render_window = vtk_widget.GetRenderWindow()
        vtk_render_window.AddRenderer(vtk_renderer)
        self.vtk_interactor = vtk_widget.GetRenderWindow().GetInteractor()
        vtk_renderer.SetBackground(0.2, 0.2, 0.2)
        # Set VTK camera position
        self.vtk_camera = vtk.vtkCamera()
        self.vtk_camera.SetPosition(7.798348203139179, -14.627243436996828, 9.812118058259466)
        self.vtk_camera.SetFocalPoint(0.5922143243356399, 0.8215172523568901, 0.7217881556358339)
        self.vtk_camera.SetViewUp(-0.1623428949744522, 0.4430640903746749, 0.8816683028621229)
        vtk_renderer.SetActiveCamera(self.vtk_camera)

        # Simulation setup
        self.timeStep = 1.0 / 60.0  # s
        self.total_t = 0
        self.sim_running = False
        self.log_enabled = True

        # Open separate window for joystick widget
        joystick_window = QtWidgets.QMainWindow(self)
        joystick_window.setWindowTitle("Joystick")
        # Remove title bar buttons
        joystick_window.setWindowFlags(QtCore.Qt.Window | QtCore.Qt.WindowTitleHint | QtCore.Qt.CustomizeWindowHint)
        joystick_widget = JoystickWidget()
        joystick_window.setCentralWidget(joystick_widget)
        joystick_window.resize(400, 400)
        joystick_window.show()

        # Initialize helicopter model
        self.heliModel = HelicopterModel()
        self.heliModel.addAllActors(vtk_renderer)
        # Initialize helicopter model for visualising the estimated state
        self.heliModelEst = HelicopterModelEstimated()
        self.heliModelEst.addAllActors(vtk_renderer)
        self.heliModelEst.setState(0, 0, 0)
        # Initialize helicopter simulation
        self.heliSim = HeliSimulation(0, 0, 0, self.timeStep)
        self.disturbance = None
        # Initialize controller and kalman filter
        self.current_controller = None
        controller_list = [ManualController(), InteractiveController(joystick_widget), PolePlacementController(),
                           LqrController(), DirectPidController(),
                           CascadePidController(), MCascadePidController(),
                           TimeVariantController(), QuasistaticFlatnessController(),
                           FeedbackLinearizationController()]

        # GUI layout
        main_layout.addWidget(vtk_widget, 1)  # Set a stretch factor > 0 so that this widget takes all remaining space
        control_top_level_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(control_top_level_layout)
        main_simulation_controls = QtWidgets.QGroupBox("Simulation")
        control_top_level_layout.addWidget(main_simulation_controls)
        main_simulation_controls_layout = QtWidgets.QVBoxLayout()
        main_simulation_controls.setLayout(main_simulation_controls_layout)
        initial_state_layout = QtWidgets.QFormLayout()
        main_simulation_controls_layout.addLayout(initial_state_layout)

        def build_slider_textedit_combo(min_value, max_value, init_value, callback):
            def slider_to_value(position):
                return position / 99.0 * (max_value - min_value) + min_value

            def value_to_slider(value):
                return int((value - min_value) / (max_value - min_value) * 99)

            h_layout = QtWidgets.QHBoxLayout()
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            h_layout.addWidget(slider)
            slider.setValue(value_to_slider(init_value))
            double_box = QtWidgets.QDoubleSpinBox()
            h_layout.addWidget(double_box)
            double_box.setRange(min_value, max_value)
            double_box.setValue(init_value)

            def on_slider():
                blocking = double_box.blockSignals(True)
                double_box.setValue(slider_to_value(slider.value()))
                double_box.blockSignals(blocking)
                callback()

            def on_box():
                blocking = slider.blockSignals(True)
                slider.setValue(value_to_slider(double_box.value()))
                slider.blockSignals(blocking)
                callback()

            slider.valueChanged.connect(on_slider)
            double_box.valueChanged.connect(on_box)

            return h_layout, double_box

        travel_layout, self.init_travel_edit = build_slider_textedit_combo(-120.0, 120.0, 0.0, self.on_init_value_change)
        initial_state_layout.addRow(QtWidgets.QLabel("Travel"), travel_layout)
        elevation_layout, self.init_elevation_edit = build_slider_textedit_combo(-70.0, 70.0, 0.0, self.on_init_value_change)
        initial_state_layout.addRow(QtWidgets.QLabel("Elevation"), elevation_layout)
        pitch_layout, self.init_pitch_edit = build_slider_textedit_combo(-80.0, 80.0, 0.0, self.on_init_value_change)
        initial_state_layout.addRow(QtWidgets.QLabel("Pitch"), pitch_layout)

        # GUI for state estimation in Simulation box
        estimated_state_layout = QtWidgets.QHBoxLayout()
        main_simulation_controls_layout.addLayout(estimated_state_layout)
        self.show_estimated_state_checkbox = QtWidgets.QCheckBox("Show estimated state")
        self.show_estimated_state_checkbox.setChecked(1)
        self.show_estimated_state_checkbox.clicked.connect(self.on_show_estimated_state_click)
        estimated_state_layout.addWidget(self.show_estimated_state_checkbox)
        self.set_initial_estimated_state_button = QtWidgets.QPushButton("Set Est. State")
        estimated_state_layout.addWidget(self.set_initial_estimated_state_button)
        self.set_initial_estimated_state_button.clicked.connect(self.set_estimated_state_button_clicked)
        # GUI for Updating controller values
        simulation_update_controller_layout = QtWidgets.QHBoxLayout()
        main_simulation_controls_layout.addLayout(simulation_update_controller_layout)
        self.update_controller_button = QtWidgets.QPushButton("Update controller values")
        self.update_controller_button.clicked.connect(self.on_controller_update_button)
        simulation_update_controller_layout.addWidget(self.update_controller_button)

        logger_layout = QtWidgets.QHBoxLayout()
        main_simulation_controls_layout.addLayout(logger_layout)
        self.log_checkbox = QtWidgets.QCheckBox("Log")
        self.log_checkbox.setChecked(self.log_enabled)
        log_show_button = QtWidgets.QPushButton("Show")
        log_show_button.clicked.connect(self.on_log_show_button)
        log_store_button = QtWidgets.QPushButton("Store")
        log_store_button.clicked.connect(self.on_log_store_button)
        logger_layout.addWidget(self.log_checkbox)
        logger_layout.addWidget(log_show_button)
        logger_layout.addWidget(log_store_button)

        simulation_control_button_layout = QtWidgets.QHBoxLayout()
        main_simulation_controls_layout.addLayout(simulation_control_button_layout)
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self.on_start_button)
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.on_stop_button)
        simulation_control_button_layout.addWidget(self.start_button)
        simulation_control_button_layout.addWidget(self.stop_button)

        settings_tabs = QtWidgets.QTabWidget()
        control_top_level_layout.addWidget(settings_tabs)
        model_frame = ModelFrame(self.heliSim)
        self.trajectory_frame = TrajectoryFrame()
        self.controller_frame = ControllerFrame(controller_list)
        self.disturbance_frame = DisturbanceFrame()
        self.feedforward_frame = FeedforwardFrame()
        self.observer_frame = ObserverFrame()
        settings_tabs.addTab(model_frame, "Model")
        settings_tabs.addTab(self.trajectory_frame, "Trajectory")
        settings_tabs.addTab(self.controller_frame, "Controller")
        settings_tabs.addTab(self.disturbance_frame, "Disturbance")
        settings_tabs.addTab(self.feedforward_frame, "Feedforward")
        settings_tabs.addTab(self.observer_frame, "Observer")
        self.feedforward_method = None
        self.feedforward_model = None
        self.observer = None
        self.observer_initial_value = np.zeros(8)

        # Get the current trajectory planner
        self.current_planner_travel, self.current_planner_elevation = self.trajectory_frame.get_planner()

        # Show the window
        self.show()
        self.vtk_interactor.Initialize()

        # Create Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timerCallback)
        self.timer.start(self.timeStep * 1000)

    def set_estimated_state_button_clicked(self):
        print("estimated state button clicked")
        orientation = np.array([self.init_pitch_edit.value(), self.init_elevation_edit.value(),
                                self.init_travel_edit.value()])
        orientation = orientation / 180.0 * np.pi
        self.observer_initial_value = [orientation[0], orientation[1], orientation[2], 0, 0, 0, 0, 0]
        self.heliModelEst.setState(orientation[2], orientation[1], orientation[0])

    def on_show_estimated_state_click(self):
        if self.show_estimated_state_checkbox.checkState() == 2:
            # show_estimated_state is checked
            self.heliModelEst.setVisibility(True)
        else:
            self.heliModelEst.setVisibility(False)

    def on_init_value_change(self):
        pass

    def on_controller_update_button(self):
        self.current_controller, param_values = self.controller_frame.get_selected_controller_and_params()
        self.current_planner_travel, self.current_planner_elevation = self.trajectory_frame.get_planner()
        logger.add_planner(self.current_planner_travel, self.current_planner_elevation)
        self.current_controller.initialize(param_values)

    def on_start_button(self):
        self.current_controller, param_values = self.controller_frame.get_selected_controller_and_params()
        self.current_planner_travel, self.current_planner_elevation = self.trajectory_frame.get_planner()
        logger.add_planner(self.current_planner_travel, self.current_planner_elevation)
        self.current_controller.initialize(param_values)
        self.disturbance = self.disturbance_frame.get_disturbance()
        self.observer = self.observer_frame.get_observer(self.timeStep)
        self.observer.set_system_model_and_step_size(self.heliSim.get_model_type(), self.timeStep)
        # self.observer.set_dynamic_inertia(self.heliSim.dynamic_inertia_torque)
        if self.observer_initial_value is not None:
            # get estimated state and JUST change the angles
            est_state = self.observer.get_estimated_state()
            est_state[0:3] = self.observer_initial_value[0:3]
            self.observer.set_estimated_state(est_state)
        self.feedforward_method,  self.feedforward_model = self.feedforward_frame.get_feedforward_method_and_model()

        self.sim_running = True
        self.log_enabled = self.log_checkbox.checkState() == 2
        if self.log_enabled:
            logger.reset()
        self.stop_button.setEnabled(True)
        self.start_button.setEnabled(False)

    def on_stop_button(self):
        self.sim_running = False
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)

    def on_log_show_button(self):
        logger.show_plots()

    def on_log_store_button(self):
        logger.open_dialog_and_store()

    def timerCallback(self, *args):
        self.total_t += self.timeStep
        theta1, theta2, theta3 = self.heliModel.getState()

        if self.sim_running:
            t = self.heliSim.get_current_time()
            x = self.heliSim.get_current_state()
            # Get feed-forward output
            # feed_forward_method = self.controller_frame.get_selected_feed_forward_method()
            e_and_derivatives = self.current_planner_elevation.eval(t)
            lambda_and_derivatives = self.current_planner_travel.eval(t)
            # The trajectory planners emit degrees, so we need to convert to rad
            e_and_derivatives = e_and_derivatives / 180 * np.pi
            lambda_and_derivatives = lambda_and_derivatives / 180 * np.pi
            # get current disturbance
            current_disturbance = self.disturbance.eval(t)

            if self.feedforward_method == FeedForwardMethod.STATIC:
                Vf_ff, Vb_ff = compute_feed_forward_static(e_and_derivatives, lambda_and_derivatives)
            elif self.feedforward_method == FeedForwardMethod.FLATNESS:
                Vf_ff, Vb_ff = compute_feed_forward_flatness(self.feedforward_model,
                                                             e_and_derivatives, lambda_and_derivatives)
            else:
                Vf_ff = 0
                Vb_ff = 0
            # Get controller output
            if not val.validation_enabled:
                Vf_controller, Vb_controller = self.current_controller.control(t, x, e_and_derivatives, lambda_and_derivatives)
            else:
                Vf_controller, Vb_controller = val.convV(val.calcInputs_numeric(dl=val.dl, e=val.e, rad=False))[0:2]

            # Add feed-forward and controller
            Vf = Vf_ff + Vf_controller
            Vb = Vb_ff + Vb_controller
            # Call observer object
            x_estimated_state, noisy_input, noisy_output, cov_matrix = self.observer.calc_observation(t, x, [Vf, Vb])
            # Log data
            if self.log_enabled:
                logger.add_frame(t, x, [Vf_ff, Vb_ff], [Vf_controller, Vb_controller],
                                 e_and_derivatives, lambda_and_derivatives, x_estimated_state, noisy_input,
                                 noisy_output, cov_matrix)
            # Calculate next simulation step
            p, e, lamb, dp, de, dlamb, f_speed, b_speed = self.heliSim.calc_step(Vf, Vb, current_disturbance)
            self.heliModel.setState(lamb, e, p)
            # This Kalman filter visualization is always one step behind the main simulation.
            # This was corrected in the logger, but was not in this visualization because for the human eye
            # it doesn't make such a big difference
            self.heliModelEst.setState(x_estimated_state[2], x_estimated_state[1], x_estimated_state[0])
        else:
            if not val.validation_enabled:
                orientation = np.array([self.init_pitch_edit.value(), self.init_elevation_edit.value(), self.init_travel_edit.value()])
                orientation = orientation / 180.0 * np.pi
                self.heliModel.setState(orientation[2], orientation[1], orientation[0])
                self.heliSim.set_current_state_and_time([orientation[0], orientation[1], orientation[2], 0, 0, 0, 0, 0])
            else:
                p = val.calcInputs_numeric(val.dl, val.e, rad=False)[2]
                orientation = np.array([p, val.e, 0, 0, 0, val.dl, 0, 0])/ 180.0 * np.pi
                self.heliModel.setState(orientation[2], orientation[1], orientation[0])
                self.heliSim.set_current_state_and_time([orientation[0], orientation[1], orientation[2], orientation[3], orientation[4], orientation[5], orientation[6], orientation[7]])

        # # handle vtk camera
        # pos = self.vtk_camera.GetPosition()
        # foc = self.vtk_camera.GetFocalPoint()
        # view_up = self.vtk_camera.GetViewUp()
        # print("pos = " + str(pos))
        # print("foc = " + str(foc))
        # print("view_up = " + str(view_up))

        self.vtk_interactor.Render()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = mainWindow()
    result = app.exec_()
    os._exit(result)  # hard kill because the gamepad thread might block
