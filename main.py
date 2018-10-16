import sys
import vtk
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import Qt
import numpy as np
from HelicopterModel import HelicopterModel
from HeliSimulation import HeliSimulation
from HeliControl import HeliControl
from HeliKalman import HeliKalmanFilter

from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

class mainWindow(Qt.QMainWindow):
    def __init__(self, parent = None):
        Qt.QMainWindow.__init__(self, parent)

        #Create Gui
        self.frame = Qt.QFrame()
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
        self.sliderTheta2 = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.sliderTheta3 = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.inputVf = QtWidgets.QLineEdit("0")
        self.inputVf.setValidator(QtGui.QDoubleValidator())
        self.inputVb = QtWidgets.QLineEdit("0")
        self.inputVb.setValidator(QtGui.QDoubleValidator())
        ##vtkWidget##
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)

        self.vl.addWidget(self.vtkWidget)
        self.vl.addWidget(self.radioButton_man)
        self.vl.addWidget(self.sliderTheta1)
        self.vl.addWidget(self.sliderTheta2)
        self.vl.addWidget(self.sliderTheta3)
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
        print(self.timeStep)
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

        #Inititalize Window, Interactor, Renderer, Layout
        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)
        self.ren.ResetCamera()
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
            theta1, theta2, theta3, dtheta1, dtheta2, dtheta3 = self.heliSim.calcStep(Vf, Vb)
            self.heliModel.setState(theta1, theta2, theta3)

        if self.progMode == "s_a":
            t = self.heliSim.getCurrentTime()
            x  = self.heliSim.getCurrentState()
            #Get controller output
            Vf, Vb = self.ctrlObj.control(t, x)
            #Call kalman filter function
            self.kalmanObj.kalman_compute(t, x, [Vf, Vb])
            #Calculate next simulation step
            theta1, theta2, theta3, dtheta1, dtheta2, dtheta3 = self.heliSim.calcStep(Vf, Vb)
            self.heliModel.setState(theta1, theta2, theta3)

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



if __name__ == "__main__":
    app = Qt.QApplication(sys.argv)
    window = mainWindow()
    sys.exit(app.exec_())
