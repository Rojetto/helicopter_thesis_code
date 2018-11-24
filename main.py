import matplotlib
matplotlib.use('Qt5Agg')

import logger
from helicontrollers.ManualController import ManualController
from helicontrollers.CascadePidController import CascadePidController
from helicontrollers.LqrController import LqrController

from ControllerFrame import ControllerFrame
from ModelFrame import ModelFrame
from TrajectoryFrame import TrajectoryFrame
from helicontrollers.DirectPidController import DirectPidController
from helicontrollers.PolePlacementController import PolePlacementController

from HelicopterModel import HelicopterModel
from HeliSimulation import HeliSimulation
from HeliKalman import HeliKalmanFilter
from MyQVTKRenderWindowInteractor import QVTKRenderWindowInteractor

import sys
import vtk
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import Qt
import numpy as np


class mainWindow(Qt.QMainWindow):
    def __init__(self, parent=None):
        Qt.QMainWindow.__init__(self, parent)

        # GUI setup
        frame = Qt.QFrame()
        main_layout = Qt.QVBoxLayout()
        frame.setLayout(main_layout)
        self.setCentralWidget(frame)

        # VTK setup
        vtk_widget = QVTKRenderWindowInteractor(frame)
        vtk_renderer = vtk.vtkRenderer()
        vtk_render_window = vtk_widget.GetRenderWindow()
        vtk_render_window.AddRenderer(vtk_renderer)
        self.vtk_interactor = vtk_widget.GetRenderWindow().GetInteractor()
        vtk_renderer.SetBackground(0.2, 0.2, 0.2)

        # Simulation setup
        self.timeStep = 1.0 / 60.0  # s
        self.total_t = 0
        self.sim_running = False
        self.log_enabled = False

        # Initialize helicopter model
        self.heliModel = HelicopterModel()
        self.heliModel.addAllActors(vtk_renderer)
        # Initialize helicopter simulation
        self.heliSim = HeliSimulation(0, 0, 0, self.timeStep)
        # Initialize controller and kalman filter
        self.current_controller = None
        self.kalmanObj = HeliKalmanFilter()
        controller_list = [ManualController(), PolePlacementController(), LqrController(), DirectPidController(), CascadePidController()]

        # GUI layout
        main_layout.addWidget(vtk_widget, 1)  # Set a stretch factor > 0 so that this widget takes all remaining space
        control_top_level_layout = Qt.QHBoxLayout()
        main_layout.addLayout(control_top_level_layout)
        main_simulation_controls = QtWidgets.QGroupBox("Simulation")
        control_top_level_layout.addWidget(main_simulation_controls)
        main_simulation_controls_layout = Qt.QVBoxLayout()
        main_simulation_controls.setLayout(main_simulation_controls_layout)
        initial_state_layout = QtWidgets.QFormLayout()
        main_simulation_controls_layout.addLayout(initial_state_layout)

        def build_slider_textedit_combo(min_value, max_value, init_value, callback):
            def slider_to_value(position):
                return position / 99.0 * (max_value - min_value) + min_value

            def value_to_slider(value):
                return int((value - min_value) / (max_value - min_value) * 99)

            h_layout = Qt.QHBoxLayout()
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

        travel_layout, self.init_travel_edit = build_slider_textedit_combo(-170.0, 170.0, 0.0, self.on_init_value_change)
        initial_state_layout.addRow(QtWidgets.QLabel("Travel"), travel_layout)
        elevation_layout, self.init_elevation_edit = build_slider_textedit_combo(-70.0, 70.0, 0.0, self.on_init_value_change)
        initial_state_layout.addRow(QtWidgets.QLabel("Elevation"), elevation_layout)
        pitch_layout, self.init_pitch_edit = build_slider_textedit_combo(-80.0, 80.0, 0.0, self.on_init_value_change)
        initial_state_layout.addRow(QtWidgets.QLabel("Pitch"), pitch_layout)
        simulation_update_controller_layout = QtWidgets.QHBoxLayout()
        main_simulation_controls_layout.addLayout(simulation_update_controller_layout)
        self.update_controller_button = QtWidgets.QPushButton("Update controller values")
        self.update_controller_button.clicked.connect(self.on_controller_update_button)
        simulation_update_controller_layout.addWidget(self.update_controller_button)
        simulation_control_button_layout = QtWidgets.QHBoxLayout()
        main_simulation_controls_layout.addLayout(simulation_control_button_layout)
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self.on_start_button)
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.on_stop_button)
        self.log_checkbox = QtWidgets.QCheckBox("Log")
        simulation_control_button_layout.addWidget(self.start_button)
        simulation_control_button_layout.addWidget(self.stop_button)
        simulation_control_button_layout.addWidget(self.log_checkbox)
        settings_tabs = QtWidgets.QTabWidget()
        control_top_level_layout.addWidget(settings_tabs)
        model_frame = ModelFrame(self.heliSim)
        self.trajectory_frame = TrajectoryFrame()
        self.controller_frame = ControllerFrame(controller_list)
        settings_tabs.addTab(model_frame, "Model")
        settings_tabs.addTab(self.trajectory_frame, "Trajectory")
        settings_tabs.addTab(self.controller_frame, "Controller")

        # Show the window
        self.show()
        self.vtk_interactor.Initialize()

        # Create Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timerCallback)
        self.timer.start(self.timeStep * 1000)

    def on_init_value_change(self):
        pass

    def on_controller_update_button(self):
        self.current_controller, param_values = self.controller_frame.get_selected_controller_and_params()
        op_travel, op_elevation = self.trajectory_frame.get_operating_point()
        self.current_controller.initialize([op_travel, op_elevation], param_values)

    def on_start_button(self):
        self.current_controller, param_values = self.controller_frame.get_selected_controller_and_params()
        op_travel, op_elevation = self.trajectory_frame.get_operating_point()
        self.current_controller.initialize([op_travel, op_elevation], param_values)

        self.sim_running = True
        self.log_enabled = self.log_checkbox.checkState() == 2
        self.stop_button.setEnabled(True)
        self.start_button.setEnabled(False)

    def on_stop_button(self):
        self.sim_running = False
        if self.log_enabled:
            logger.finish()
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)

    def timerCallback(self, *args):
        self.total_t += self.timeStep
        theta1, theta2, theta3 = self.heliModel.getState()

        if self.sim_running:
            t = self.heliSim.get_current_time()
            x = self.heliSim.get_current_state()
            # Get controller output
            Vf, Vb = self.current_controller.control(t, x)
            # Call kalman filter function
            self.kalmanObj.kalman_compute(t, x, [Vf, Vb])
            # Log data
            if self.log_enabled:
                logger.add_frame(t, x, [Vf, Vb])
            # Calculate next simulation step
            p, e, lamb, dp, de, dlamb = self.heliSim.calc_step(Vf, Vb)
            self.heliModel.setState(lamb, e, p)
        else:
            orientation = np.array([self.init_pitch_edit.value(), self.init_elevation_edit.value(), self.init_travel_edit.value()])
            orientation = orientation / 180.0 * np.pi
            self.heliModel.setState(orientation[2], orientation[1], orientation[0])
            self.heliSim.set_current_state_and_time([orientation[0], orientation[1], orientation[2], 0, 0, 0])

        self.vtk_interactor.Render()


if __name__ == "__main__":
    app = Qt.QApplication(sys.argv)
    window = mainWindow()
    sys.exit(app.exec_())
