from PyQt5 import QtWidgets
import numpy as np


class OperatingPointFrame(QtWidgets.QFrame):
    def __init__(self):
        QtWidgets.QFrame.__init__(self)

        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)

        layout.addWidget(QtWidgets.QLabel("Static operating point:"), 0, 0)
        self.op_travel = QtWidgets.QDoubleSpinBox()
        self.op_elevation = QtWidgets.QDoubleSpinBox()
        layout.addWidget(QtWidgets.QLabel("Travel"), 0, 1)
        layout.addWidget(self.op_travel, 0, 2)
        layout.addWidget(QtWidgets.QLabel("Elevation"), 0, 3)
        layout.addWidget(self.op_elevation, 0, 4)

    def get_operating_point(self):
        """
        :return: travel (rad), elevation (rad)
        """
        return self.op_travel.value() / 180.0 * np.pi, self.op_elevation.value() / 180.0 * np.pi