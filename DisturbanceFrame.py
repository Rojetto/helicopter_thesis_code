from PyQt5 import QtWidgets, QtGui
import numpy as np
import Planner
from Disturbance import Disturbance, DisturbanceStep, DisturbanceSinus, NoDisturbance


class DisturbanceFrame(QtWidgets.QFrame):
    def __init__(self):
        # QtWidgets.QFrame.__init__(self)
        super().__init__()

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setFixedHeight(200)

        self.disturbance_combo = QtWidgets.QComboBox()
        self.disturbance_frame_stack = QtWidgets.QStackedWidget()

        # fill the QStackWidget manually with Frames. If this becomes too tedious and confusing
        # then maybe switch to doing something with more abstraction

        # Disturbance step
        self.disturbance_combo.insertItem(1, "Disturbance Step")
        disturbance_step_frame = QtWidgets.QFrame()
        disturbance_step_layout = QtWidgets.QFormLayout()
        disturbance_step_frame.setLayout(disturbance_step_layout)

        self.disturbance_step_t_start = QtWidgets.QDoubleSpinBox()
        self.disturbance_step_t_start.setValue(1)
        self.disturbance_step_z0 = QtWidgets.QDoubleSpinBox()
        self.disturbance_step_z0.setValue(0)
        self.disturbance_step_z_step = QtWidgets.QDoubleSpinBox()
        self.disturbance_step_z_step.setValue(20)
        self.disturbance_step_type = QtWidgets.QComboBox()
        self.disturbance_step_type.addItems(["p", "e", "lambda", "f", "b"])
        self.disturbance_step_z_step.setMinimum(-99999)

        disturbance_step_layout.addRow(QtWidgets.QLabel("Step time t_start"),
                                             self.disturbance_step_t_start)
        disturbance_step_layout.addRow(QtWidgets.QLabel("Disturbance initial value"),
                                             self.disturbance_step_z0)
        disturbance_step_layout.addRow(QtWidgets.QLabel("Disturbance step height"),
                                             self.disturbance_step_z_step)
        disturbance_step_layout.addRow(QtWidgets.QLabel("Disturbance point of application"),
                                       self.disturbance_step_type)

        self.disturbance_frame_stack.addWidget(disturbance_step_frame)

        # Sinus Disturbance
        self.disturbance_combo.insertItem(2, "Sinus Disturbance")
        disturbance_sin_frame = QtWidgets.QFrame()
        disturbance_sin_layout = QtWidgets.QFormLayout()
        disturbance_sin_frame.setLayout(disturbance_sin_layout)

        self.disturbance_sin_t_start = QtWidgets.QDoubleSpinBox()
        self.disturbance_sin_t_start.setValue(1)
        self.disturbance_sin_z_offset = QtWidgets.QDoubleSpinBox()
        self.disturbance_sin_z_offset.setValue(0)
        self.disturbance_sin_z_amplitude = QtWidgets.QDoubleSpinBox()
        self.disturbance_sin_z_amplitude.setValue(0.5)
        self.disturbance_sin_z_frequency = QtWidgets.QDoubleSpinBox()
        self.disturbance_sin_z_frequency.setValue(1)
        self.disturbance_sin_type = QtWidgets.QComboBox()
        self.disturbance_sin_type.addItems(["p", "e", "lambda", "f", "b"])

        disturbance_sin_layout.addRow(QtWidgets.QLabel("Start time"),
                                       self.disturbance_sin_t_start)
        disturbance_sin_layout.addRow(QtWidgets.QLabel("DC Value / Offset"),
                                       self.disturbance_sin_z_offset)
        disturbance_sin_layout.addRow(QtWidgets.QLabel("Amplitude"),
                                       self.disturbance_sin_z_amplitude)
        disturbance_sin_layout.addRow(QtWidgets.QLabel("Frequency (Hz)"),
                                       self.disturbance_sin_z_frequency)
        disturbance_sin_layout.addRow(QtWidgets.QLabel("Disturbance point of application"),
                                       self.disturbance_sin_type)

        self.disturbance_frame_stack.addWidget(disturbance_sin_frame)

        # Add "No Disturbance"

        self.disturbance_combo.insertItem(3, "No Disturbance")
        no_disturbance_frame = QtWidgets.QFrame()
        no_disturbance_layout = QtWidgets.QFormLayout()
        no_disturbance_frame.setLayout(no_disturbance_layout)

        no_disturbance_layout.addRow(QtWidgets.QLabel("No Disturbance"),
                                      QtWidgets.QLabel(""))

        self.disturbance_frame_stack.addWidget(no_disturbance_frame)

        # Finishing touches
        scroll_area.setWidget(self.disturbance_frame_stack)
        main_layout.addWidget(scroll_area, 1)
        main_layout.addWidget(self.disturbance_combo)
        self.disturbance_combo.currentIndexChanged.connect(self.on_disturbance_combo_select)
        self.disturbance_combo.setCurrentIndex(2)

    def on_disturbance_combo_select(self):
        # print("combo select: " + str(self.trajectory_combo.currentIndex()))
        # print("count = " + str(self.trajectory_frame_stack.count()))
        self.disturbance_frame_stack.setCurrentIndex(self.disturbance_combo.currentIndex())

    def get_disturbance(self):
        """:return disturbance object """
        combo_idx = self.disturbance_combo.currentIndex()
        # the chosen combo entry defines the type of planner that is returned
        if combo_idx == 0:
            # print("Disturbance step")
            t_start = self.disturbance_step_t_start.value()
            z0 = self.disturbance_step_z0.value()
            zf = self.disturbance_step_z_step.value()
            point_of_application = self.disturbance_step_type.currentText()
            disturbance = DisturbanceStep(t_start, z0, zf, point_of_application)
        elif combo_idx == 1:
            # print("Disturbance Sinus")
            t_start = self.disturbance_sin_t_start.value()
            z_offset = self.disturbance_sin_z_offset.value()
            z_amplitude = self.disturbance_sin_z_amplitude.value()
            z_frequency = self.disturbance_sin_z_frequency.value()
            point_of_application = self.disturbance_sin_type.currentText()
            disturbance = DisturbanceSinus(t_start, z_offset, z_amplitude, z_frequency, point_of_application)
        elif combo_idx == 2:
            print("No Disturbance")
            disturbance = NoDisturbance()
        return disturbance