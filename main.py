import sys
import vtk
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import Qt
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plot
from HelicopterModel import HelicopterModel
from HeliSimulation import HeliSimulation
from HeliControl import HeliControl, ControlMethod
from HeliKalman import HeliKalmanFilter

from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

class mainWindow(Qt.QMainWindow):
    def __init__(self, parent = None):
        Qt.QMainWindow.__init__(self, parent)

        #Create Gui
        self.frame = Qt.QFrame()
        self.manualGridVL = Qt.QGridLayout()
        self.vl = Qt.QVBoxLayout()
        ##Other Widgets##
        self.radioButton_man = QtWidgets.QRadioButton("Manual Mode", self)
        self.radioButton_man.setChecked(True)
        self.radioButton_man.toggled.connect(lambda : self.radioBtnState(self.radioButton_man))
        self.radioButton_sim = QtWidgets.QRadioButton("Simulation (Manual) Mode", self)
        self.radioButton_sim.toggled.connect(lambda: self.radioBtnState(self.radioButton_sim))
        self.radioButton_auto = QtWidgets.QRadioButton("Simulation (Auto) Mode", self)
        self.radioButton_auto.toggled.connect(lambda: self.radioBtnState(self.radioButton_auto))
        self.sliderTheta1 = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.sliderTheta1edit = QtWidgets.QLineEdit("0.0")
        self.sliderTheta1edit.setValidator(QtGui.QDoubleValidator())
        self.sliderTheta1.sliderReleased.connect(lambda: self.sliderTheta1edit.setText(
            str(self.sliderTheta1.value() / 100 * 2 * np.pi)))
        #ATTENTION: ValueErrors can easily happen here
        self.sliderTheta1edit.editingFinished.connect(lambda : self.sliderTheta1.setValue(
                                                        float(self.sliderTheta1edit.text()) * 100 / (2*np.pi)))
        self.sliderTheta2 = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.sliderTheta2edit = QtWidgets.QLineEdit("0.0")
        self.sliderTheta2edit.setValidator(QtGui.QDoubleValidator())
        self.sliderTheta2.sliderReleased.connect(lambda: self.sliderTheta2edit.setText(
            str(self.sliderTheta2.value() / 100 * 2 * np.pi)))
        self.sliderTheta2edit.editingFinished.connect(lambda: self.sliderTheta2.setValue(
            float(self.sliderTheta2edit.text()) * 100 / (2 * np.pi)))
        self.sliderTheta3 = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.sliderTheta3edit = QtWidgets.QLineEdit("0.0")
        self.sliderTheta3edit.setValidator(QtGui.QDoubleValidator())
        self.sliderTheta3.sliderReleased.connect(lambda: self.sliderTheta3edit.setText(
            str(self.sliderTheta3.value() / 100 * 2 * np.pi)))
        self.sliderTheta3edit.editingFinished.connect(lambda: self.sliderTheta3.setValue(
            float(self.sliderTheta3edit.text()) * 100 / (2 * np.pi)))
        self.theta1OPedit = QtWidgets.QLineEdit("0.0")
        self.theta1OPedit.setValidator(QtGui.QDoubleValidator())
        self.theta2OPedit = QtWidgets.QLineEdit("0.0")
        self.theta2OPedit.setValidator(QtGui.QDoubleValidator())
        self.theta3OPedit = QtWidgets.QLineEdit("0.0")
        self.theta3OPedit.setValidator(QtGui.QDoubleValidator())
        self.btnSetSimState = QtWidgets.QPushButton("Set this state to simulation")
        self.btnSetSimState.clicked.connect(self.btnSetSimState_clicked)
        self.btnSetOP = QtWidgets.QPushButton("Set operational point")
        self.btnSetOP.clicked.connect(self.btnSetOP_clicked)
        self.inputVf = QtWidgets.QLineEdit("0")
        self.inputVf.setValidator(QtGui.QDoubleValidator())
        self.inputVb = QtWidgets.QLineEdit("0")
        self.inputVb.setValidator(QtGui.QDoubleValidator())
        self.labelLambda = QtWidgets.QLabel("Lambda")
        self.labelElevation = QtWidgets.QLabel("Elevation")
        self.labelPitch = QtWidgets.QLabel("Pitch")
        ##vtkWidget##
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)

        self.manualGridVL.addWidget(self.sliderTheta1, 0, 1)
        self.manualGridVL.addWidget(self.sliderTheta1edit, 0, 2)
        self.manualGridVL.addWidget(self.sliderTheta2, 1, 1)
        self.manualGridVL.addWidget(self.sliderTheta2edit, 1, 2)
        self.manualGridVL.addWidget(self.sliderTheta3, 2, 1)
        self.manualGridVL.addWidget(self.sliderTheta3edit, 2, 2)
        self.manualGridVL.addWidget(self.theta1OPedit, 0, 3)
        self.manualGridVL.addWidget(self.theta2OPedit, 1, 3)
        self.manualGridVL.addWidget(self.theta3OPedit, 2, 3)
        self.manualGridVL.addWidget(self.labelLambda, 0, 0)
        self.manualGridVL.addWidget(self.labelElevation, 1, 0)
        self.manualGridVL.addWidget(self.labelPitch, 2, 0)

        self.vl.addWidget(self.vtkWidget)
        self.vl.addWidget(self.radioButton_man)
        self.vl.addLayout(self.manualGridVL)
        self.vl.addWidget(self.btnSetSimState)
        self.vl.addWidget(self.btnSetOP)
        self.vl.addWidget(self.radioButton_sim)
        self.vl.addWidget(self.inputVf)
        self.vl.addWidget(self.inputVb)
        self.vl.addWidget(self.radioButton_auto)

        self.ren = vtk.vtkRenderer()
        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        #Settings
        self.ren.SetBackground(0.2, 0.2, 0.2)
        self.timeStep = 1/60 * 1000 #ms
        self.total_t = 0
        self.progMode = 'm'

        #Initialize helicopter model
        self.heliModel = HelicopterModel()
        self.heliModel.addAllActors(self.ren)
        #Initialize helicopter simulation
        self.heliSim = HeliSimulation(0, 0, 0, self.timeStep/1000)
        #Initialize controller and kalman filter
        self.ctrlObj = HeliControl()
        self.kalmanObj = HeliKalmanFilter()

        # TEMP logging
        self.nr_log_entries = int(4 / (self.timeStep / 1000))  # First number is the time in seconds
        self.log_ts = np.empty(self.nr_log_entries)
        self.log_xs = np.empty((self.nr_log_entries, 6))
        self.log_us = np.empty((self.nr_log_entries, 2))
        self.log_index = 0
        self.log_enabled = False
        # TEMP logging

        #Inititalize Window, Interactor, Renderer, Layout
        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)
        self.show()
        self.iren.Initialize()

        # Create Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timerCallback)
        self.timer.start(self.timeStep)

    def timerCallback(self, *args):
        self.total_t += self.timeStep / 1000
        theta1, theta2, theta3 = self.heliModel.getState()

        if self.progMode == "m":
            #!!!!ATTENTION!!! ValueError can easily happen here
            self.heliModel.setState(self.sliderTheta1.value()/100 * 2*np.pi,
                                    self.sliderTheta2.value() / 100 * 2 * np.pi,
                                    self.sliderTheta3.value() / 100 * 2 * np.pi)
        if self.progMode == "s_m":
            #Calculate Simulation step
            try:
                Vf = float(self.inputVf.text())
                Vb = float(self.inputVb.text())
            except ValueError:
                #print("[DEBUG]ValueError in main.py")
                Vf = 0
                Vb = 0
            p, e, lamb, dp, de, dlamb = self.heliSim.calcStep(Vf, Vb)
            self.heliModel.setState(lamb, e, p)

        if self.progMode == "s_a":
            t = self.heliSim.getCurrentTime()
            x  = self.heliSim.getCurrentState()
            #Get controller output
            Vf, Vb = self.ctrlObj.control(t, x)
            #Call kalman filter function
            self.kalmanObj.kalman_compute(t, x, [Vf, Vb])
            #Calculate next simulation step
            p, e, lamb, dp, de, dlamb = self.heliSim.calcStep(Vf, Vb)
            self.heliModel.setState(lamb, e, p)

            # TEMP logging
            if self.log_enabled:
                self.log_ts[self.log_index] = self.heliSim.getCurrentTime()
                self.log_xs[self.log_index] = self.heliSim.getCurrentState()
                self.log_us[self.log_index, 0] = Vf
                self.log_us[self.log_index, 1] = Vb

                self.log_index += 1

                if self.log_index >= self.nr_log_entries:
                    self.log_enabled = False
                    print("Finished logging")
                    self.log_ts -= self.log_ts[0]

                    plot.close("all")

                    plot.figure("System state")
                    plot.subplot(311)
                    plot.plot(self.log_ts, self.log_xs[:, 0])
                    plot.axhline(self.ctrlObj.operatingPoint[0])
                    plot.grid()
                    plot.subplot(312)
                    plot.plot(self.log_ts, self.log_xs[:, 1])
                    plot.axhline(self.ctrlObj.operatingPoint[1])
                    plot.grid()
                    plot.subplot(313)
                    plot.plot(self.log_ts, self.log_xs[:, 2])
                    plot.axhline(self.ctrlObj.operatingPoint[2])
                    plot.grid()
                    plot.tight_layout()

                    plot.figure("Controller output")
                    plot.plot(self.log_ts, self.log_us[:, 0])
                    plot.plot(self.log_ts, self.log_us[:, 1])
                    plot.grid()
                    plot.tight_layout()

                    plot.show()
            # TEMP logging

        self.iren.Render()

    def radioBtnState(self, b):
        if b.text() == "Manual Mode" and b.isChecked(): #let's ignore the unchecking events and see what happens
            self.progMode = 'm'
            print("Manual Mode was selected")

        if b.text() == "Simulation (Manual) Mode" and b.isChecked():
            self.progMode = 's_m'
            print("Simulation (Manual) Mode was selected")

        if b.text() == "Simulation (Auto) Mode" and b.isChecked():
            self.progMode = 's_a'
            print("Simulation (Auto) Mode was selected")
            # TEMP logging
            self.log_index = 0
            self.log_enabled = True
            # TEMP logging

    def btnSetSimState_clicked(self):
        self.heliSim.setCurrentState([float(self.sliderTheta3edit.text()),
                                      float(self.sliderTheta2edit.text()),
                                      float(self.sliderTheta1edit.text()),
                                      0, 0, 0])

    def btnSetOP_clicked(self):
        self.ctrlObj.setOperatingPoint([float(self.theta3OPedit.text()), float(self.theta2OPedit.text()),
                                        float(self.theta1OPedit.text())])


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
        self.radio_pid = QtWidgets.QRadioButton("PID", self)
        self.radio_pid.toggled.connect(self.on_radio_pid_toggle)
        self.layout.addWidget(self.radio_pid, 3, 0)

        self.poles = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                      QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self)]

        self.layout.addWidget(QtWidgets.QLabel("Poles:"), 0, 1)
        for i, pole_edit in enumerate(self.poles):
            pole_edit.setRange(-100, -0.01)
            pole_edit.setValue(self.heli_controller.feedback_poles[i])
            pole_edit.valueChanged.connect(self.on_pole_edit)
            self.layout.addWidget(pole_edit, 0, i + 2)

        self.q_diagonal = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self),
                           QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self)]

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
        self.layout.addWidget(QtWidgets.QLabel("PID travel"), 4, 1)

        self.pid_e_gains = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self)]
        self.pid_lambda_gains = [QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self), QtWidgets.QDoubleSpinBox(self)]

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
        self.heli_controller.setElevationPidGains(pid_e_gains)
        self.heli_controller.setTravelPidGains(pid_lambda_gains)

    def on_radio_poles_toggle(self, checked):
        if checked:
            self.heli_controller.setControlMethod(ControlMethod.POLES)

    def on_radio_lqr_toggle(self, checked):
        if checked:
            self.heli_controller.setControlMethod(ControlMethod.LQR)

    def on_radio_pid_toggle(self, checked):
        if checked:
            self.heli_controller.setControlMethod(ControlMethod.PID)


if __name__ == "__main__":
    app = Qt.QApplication(sys.argv)
    window = mainWindow()
    control_window = ControlWindow(window.ctrlObj)
    sys.exit(app.exec_())
