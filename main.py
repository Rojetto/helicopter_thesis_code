import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plot
from HelicopterModel import HelicopterModel
from HeliSimulation import HeliSimulation
from HeliControl import HeliControl, ControlMethod
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

        # GUI Layout
        frame = Qt.QFrame()
        main_layout = Qt.QVBoxLayout()
        frame.setLayout(main_layout)
        self.setCentralWidget(frame)

        vtk_widget = QVTKRenderWindowInteractor(frame)
        main_layout.addWidget(vtk_widget)
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
        model_frame = QtWidgets.QFrame()
        trajectory_frame = QtWidgets.QFrame()
        controller_frame = QtWidgets.QFrame()
        settings_tabs.addTab(model_frame, "Model")
        settings_tabs.addTab(trajectory_frame, "Trajectory")
        settings_tabs.addTab(controller_frame, "Controller")

        # VTK setup
        vtk_renderer = vtk.vtkRenderer()
        vtk_render_window = vtk_widget.GetRenderWindow()
        vtk_render_window.AddRenderer(vtk_renderer)
        self.vtk_interactor = vtk_widget.GetRenderWindow().GetInteractor()
        vtk_renderer.SetBackground(0.2, 0.2, 0.2)

        # Simulation setup
        self.timeStep = 1 / 60 * 1000  # ms
        self.total_t = 0
        self.sim_running = False

        # Initialize helicopter model
        self.heliModel = HelicopterModel()
        self.heliModel.addAllActors(vtk_renderer)
        # Initialize helicopter simulation
        self.heliSim = HeliSimulation(0, 0, 0, self.timeStep / 1000)
        # Initialize controller and kalman filter
        self.ctrlObj = HeliControl()
        self.kalmanObj = HeliKalmanFilter()

        # Show the window
        self.show()
        self.vtk_interactor.Initialize()

        # Create Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timerCallback)
        self.timer.start(self.timeStep)

    def on_init_value_change(self):
        pass

    def on_start_button(self):
        self.sim_running = True
        self.stop_button.setEnabled(True)
        self.start_button.setEnabled(False)

    def on_stop_button(self):
        self.sim_running = False
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)

    def timerCallback(self, *args):
        self.total_t += self.timeStep / 1000
        theta1, theta2, theta3 = self.heliModel.getState()

        if self.sim_running:
            t = self.heliSim.getCurrentTime()
            x = self.heliSim.getCurrentState()
            # Get controller output
            Vf, Vb = self.ctrlObj.control(t, x)
            # Call kalman filter function
            self.kalmanObj.kalman_compute(t, x, [Vf, Vb])
            # Calculate next simulation step
            p, e, lamb, dp, de, dlamb = self.heliSim.calcStep(Vf, Vb)
            self.heliModel.setState(lamb, e, p)
        else:
            orientation = np.array([self.init_pitch_edit.value(), self.init_elevation_edit.value(), self.init_travel_edit.value()])
            orientation = orientation / 180.0 * np.pi
            self.heliModel.setState(orientation[2], orientation[1], orientation[0])
            self.heliSim.setCurrentState([orientation[0], orientation[1], orientation[2], 0, 0, 0])

        self.vtk_interactor.Render()


class ControlWindow(Qt.QMainWindow):
    def __init__(self, heli_controller: HeliControl, parent=None):
        Qt.QMainWindow.__init__(self, parent)

        self.heli_controller = heli_controller

        self.frame = Qt.QFrame()
        self.layout = Qt.QGridLayout()

        self.radio_poles = QtWidgets.QRadioButton("Pole placement", self)
        self.radio_poles.setChecked(True)
        self.radio_poles.toggled.connect(self.on_radio_poles_toggle)
        self.layout.addWidget(self.radio_poles, 0, 0)
        self.radio_lqr = QtWidgets.QRadioButton("LQR", self)
        self.radio_lqr.toggled.connect(self.on_radio_lqr_toggle)
        self.layout.addWidget(self.radio_lqr, 1, 0)
        self.radio_pid_direct = QtWidgets.QRadioButton("PID direct", self)
        self.radio_pid_direct.toggled.connect(self.on_radio_pid_direct_toggle)
        self.layout.addWidget(self.radio_pid_direct, 4, 0)
        self.radio_pid_cascade = QtWidgets.QRadioButton("PID cascade", self)
        self.radio_pid_cascade.toggled.connect(self.on_radio_pid_cascade_toggle)
        self.layout.addWidget(self.radio_pid_cascade, 5, 0)

        self.poles = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                      QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self)]

        self.layout.addWidget(QtWidgets.QLabel("Poles:"), 0, 1)
        for i, pole_edit in enumerate(self.poles):
            pole_edit.setRange(-100, -0.01)
            pole_edit.setValue(self.heli_controller.feedback_poles[i])
            pole_edit.valueChanged.connect(self.on_pole_edit)
            self.layout.addWidget(pole_edit, 0, i + 2)

        self.q_diagonal = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                           QtWidgets.QDoubleSpinBox(self),
                           QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                           QtWidgets.QDoubleSpinBox(self)]

        self.r_diagonal = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self)]

        self.layout.addWidget(QtWidgets.QLabel("diag(Q):"), 1, 1)
        self.layout.addWidget(QtWidgets.QLabel("diag(R):"), 2, 1)

        for i, lqr_edit in enumerate(self.q_diagonal):
            lqr_edit.setRange(0.01, 100)
            lqr_edit.setValue(self.heli_controller.lqr_Q[i])
            lqr_edit.valueChanged.connect(self.on_lqr_edit)
            self.layout.addWidget(lqr_edit, 1, i + 2)
        for i, lqr_edit in enumerate(self.r_diagonal):
            lqr_edit.setRange(0.01, 100)
            lqr_edit.setValue(self.heli_controller.lqr_R[i])
            lqr_edit.valueChanged.connect(self.on_lqr_edit)
            self.layout.addWidget(lqr_edit, 2, i + 2)

        self.layout.addWidget(QtWidgets.QLabel("PID elevation"), 3, 1)
        self.layout.addWidget(QtWidgets.QLabel("PID travel-Vd"), 4, 1)
        self.layout.addWidget(QtWidgets.QLabel("PID travel-pitch"), 5, 1)
        self.layout.addWidget(QtWidgets.QLabel("PID pitch-Vd"), 6, 1)

        self.pid_e_gains = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                            QtWidgets.QDoubleSpinBox(self)]
        self.pid_lambda_gains = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                                 QtWidgets.QDoubleSpinBox(self)]
        self.pid_travel_pitch_gains = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                                       QtWidgets.QDoubleSpinBox(self)]
        self.pid_pitch_vd_gains = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                                   QtWidgets.QDoubleSpinBox(self)]

        for i, pid_edit in enumerate(self.pid_e_gains):
            pid_edit.setRange(0, 100)
            pid_edit.setValue(self.heli_controller.pid_elevation_gains[i])
            pid_edit.valueChanged.connect(self.on_pid_edit)
            self.layout.addWidget(pid_edit, 3, i + 2)
        for i, pid_edit in enumerate(self.pid_lambda_gains):
            pid_edit.setRange(0, 100)
            pid_edit.setValue(self.heli_controller.pid_travel_gains[i])
            pid_edit.valueChanged.connect(self.on_pid_edit)
            self.layout.addWidget(pid_edit, 4, i + 2)
        for i, pid_edit in enumerate(self.pid_travel_pitch_gains):
            pid_edit.setRange(0, 100)
            pid_edit.setValue(self.heli_controller.pid_travel_pitch_gains[i])
            pid_edit.valueChanged.connect(self.on_pid_edit)
            self.layout.addWidget(pid_edit, 5, i + 2)
        for i, pid_edit in enumerate(self.pid_pitch_vd_gains):
            pid_edit.setRange(0, 100)
            pid_edit.setValue(self.heli_controller.pid_pitch_vd_gains[i])
            pid_edit.valueChanged.connect(self.on_pid_edit)
            self.layout.addWidget(pid_edit, 6, i + 2)

        self.frame.setLayout(self.layout)
        self.setCentralWidget(self.frame)
        self.show()

    def on_pole_edit(self, new_pole_value):
        pole_values = [pole_edit.value() for pole_edit in self.poles]
        self.heli_controller.setFeedbackPoles(pole_values)

    def on_lqr_edit(self, new_value):
        q_diag = [lqr_edit.value() for lqr_edit in self.q_diagonal]
        r_diag = [lqr_edit.value() for lqr_edit in self.r_diagonal]
        self.heli_controller.setLqrQDiagonal(q_diag)
        self.heli_controller.setLqrRDiagonal(r_diag)

    def on_pid_edit(self, new_value):
        pid_e_gains = [pid_edit.value() for pid_edit in self.pid_e_gains]
        pid_lambda_gains = [pid_edit.value() for pid_edit in self.pid_lambda_gains]
        pid_travel_pitch_gains = [pid_edit.value() for pid_edit in self.pid_travel_pitch_gains]
        pid_pitch_vd_gains = [pid_edit.value() for pid_edit in self.pid_pitch_vd_gains]
        self.heli_controller.setElevationPidGains(pid_e_gains)
        self.heli_controller.setTravelPidGains(pid_lambda_gains)
        self.heli_controller.setTravelPitchPidGains(pid_travel_pitch_gains)
        self.heli_controller.setPitchVdPidGains(pid_pitch_vd_gains)

    def on_radio_poles_toggle(self, checked):
        if checked:
            self.heli_controller.setControlMethod(ControlMethod.POLES)

    def on_radio_lqr_toggle(self, checked):
        if checked:
            self.heli_controller.setControlMethod(ControlMethod.LQR)

    def on_radio_pid_direct_toggle(self, checked):
        if checked:
            self.heli_controller.setControlMethod(ControlMethod.PID_DIRECT)

    def on_radio_pid_cascade_toggle(self, checked):
        if checked:
            self.heli_controller.setControlMethod(ControlMethod.PID_CASCADE)


if __name__ == "__main__":
    app = Qt.QApplication(sys.argv)
    window = mainWindow()
    control_window = ControlWindow(window.ctrlObj)
    sys.exit(app.exec_())
