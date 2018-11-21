from PyQt5 import QtWidgets, QtGui
import numpy as np


class TrajectoryFrame(QtWidgets.QFrame):
    def __init__(self):
        # QtWidgets.QFrame.__init__(self)
        super().__init__()

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setMinimumHeight(200)

        self.trajectory_combo = QtWidgets.QComboBox()
        self.trajectory_frame_stack = QtWidgets.QStackedWidget()

        # fill the QStackWidget manually with Frames. If this becomes too tedious and confusing
        # then maybe switch to doing something with more abstraction
        # Fixed Operating Point
        self.trajectory_combo.insertItem(1, "Fixed Operating Point")
        fixed_op_frame = QtWidgets.QFrame()
        fixed_op_layout = QtWidgets.QGridLayout()
        fixed_op_frame.setLayout(fixed_op_layout)
        fixed_op_layout.addWidget(QtWidgets.QLabel("Static operating point:"), 0, 0)
        self.fixed_op_travel = QtWidgets.QDoubleSpinBox()
        self.fixed_op_elevation = QtWidgets.QDoubleSpinBox()
        fixed_op_layout.addWidget(QtWidgets.QLabel("Travel (degree)"), 0, 1)
        fixed_op_layout.addWidget(self.fixed_op_travel, 0, 2)
        fixed_op_layout.addWidget(QtWidgets.QLabel("Elevation (degree)"), 0, 3)
        fixed_op_layout.addWidget(self.fixed_op_elevation, 0, 4)
        self.trajectory_frame_stack.addWidget(fixed_op_frame)

        # Operating Point Transfer
        self.trajectory_combo.insertItem(2, "Operating Point Transfer (Polynomial)")
        op_transfer_polynomial_frame = QtWidgets.QFrame()
        op_transfer_polynomial_layout = QtWidgets.QFormLayout()
        op_transfer_polynomial_frame.setLayout(op_transfer_polynomial_layout)

        self.op_transfer_polynomial_start_travel = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_start_elevation = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_end_travel = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_end_elevation = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_max_derivative_order = QtWidgets.QLineEdit("4")
        self.op_transfer_polynomial_max_derivative_order.setValidator(QtGui.QIntValidator(0, 999999))
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Start Travel angle"),
                                             self.op_transfer_polynomial_start_travel)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Start Elevation Angle"),
                                             self.op_transfer_polynomial_start_elevation)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("End Travel angle"),
                                             self.op_transfer_polynomial_end_travel)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("End Elevation angle"),
                                             self.op_transfer_polynomial_end_elevation)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Max. derivative order d"),
                                             self.op_transfer_polynomial_max_derivative_order)
        self.trajectory_frame_stack.addWidget(op_transfer_polynomial_frame)

        scroll_area.setWidget(self.trajectory_frame_stack)
        main_layout.addWidget(scroll_area, 1)
        main_layout.addWidget(self.trajectory_combo)
        self.trajectory_combo.currentIndexChanged.connect(self.on_trajectory_combo_select)
        self.trajectory_combo.setCurrentIndex(1)

    def on_trajectory_combo_select(self):
        # print("combo select: " + str(self.trajectory_combo.currentIndex()))
        # print("count = " + str(self.trajectory_frame_stack.count()))
        self.trajectory_frame_stack.setCurrentIndex(self.trajectory_combo.currentIndex())

    def get_operating_point(self):
        """
        :return: travel (rad), elevation (rad)
        """
        return self.fixed_op_travel.value() / 180.0 * np.pi, self.fixed_op_elevation.value() / 180.0 * np.pi
