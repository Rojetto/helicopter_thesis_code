from PyQt5 import QtWidgets, QtGui
import numpy as np
import Planner


class TrajectoryFrame(QtWidgets.QFrame):
    def __init__(self):
        # QtWidgets.QFrame.__init__(self)
        super().__init__()

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setFixedHeight(200)

        self.trajectory_combo = QtWidgets.QComboBox()
        self.trajectory_frame_stack = QtWidgets.QStackedWidget()

        # fill the QStackWidget manually with Frames. If this becomes too tedious and confusing
        # then maybe switch to doing something with more abstraction
        # Fixed Operating Point
        # self.trajectory_combo.insertItem(1, "Fixed Operating Point")
        # fixed_op_frame = QtWidgets.QFrame()
        # fixed_op_layout = QtWidgets.QGridLayout()
        # fixed_op_frame.setLayout(fixed_op_layout)
        # fixed_op_layout.addWidget(QtWidgets.QLabel("Static operating point:"), 0, 0)
        # self.fixed_op_travel = QtWidgets.QDoubleSpinBox()
        # self.fixed_op_elevation = QtWidgets.QDoubleSpinBox()
        # fixed_op_layout.addWidget(QtWidgets.QLabel("Travel (degree)"), 0, 1)
        # fixed_op_layout.addWidget(self.fixed_op_travel, 0, 2)
        # fixed_op_layout.addWidget(QtWidgets.QLabel("Elevation (degree)"), 0, 3)
        # fixed_op_layout.addWidget(self.fixed_op_elevation, 0, 4)
        # self.trajectory_frame_stack.addWidget(fixed_op_frame)

        # Operating Point Transfer Polynomial
        self.trajectory_combo.insertItem(1, "Operating Point Transfer (Polynomial)")
        op_transfer_polynomial_frame = QtWidgets.QFrame()
        op_transfer_polynomial_layout = QtWidgets.QFormLayout()
        op_transfer_polynomial_frame.setLayout(op_transfer_polynomial_layout)

        self.op_transfer_polynomial_t0 = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_t0.setValue(0)
        self.op_transfer_polynomial_tf = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_tf.setValue(2)
        self.op_transfer_polynomial_start_travel = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_start_elevation = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_end_travel = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_end_travel.setValue(45)
        self.op_transfer_polynomial_end_elevation = QtWidgets.QDoubleSpinBox()
        self.op_transfer_polynomial_end_elevation.setValue(10)
        self.op_transfer_polynomial_max_derivative_order_travel = QtWidgets.QLineEdit("4")
        self.op_transfer_polynomial_max_derivative_order_travel.setValidator(QtGui.QIntValidator(0, 999999))
        self.op_transfer_polynomial_max_derivative_order_elevation = QtWidgets.QLineEdit("4")
        self.op_transfer_polynomial_max_derivative_order_elevation.setValidator(QtGui.QIntValidator(0, 999999))
        self.op_transfer_polynomial_prototype_checkbox = QtWidgets.QCheckBox()
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Initial Time t0"),
                                             self.op_transfer_polynomial_t0)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Final Time tf"),
                                             self.op_transfer_polynomial_tf)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Start Travel angle"),
                                             self.op_transfer_polynomial_start_travel)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Start Elevation Angle"),
                                             self.op_transfer_polynomial_start_elevation)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("End Travel angle"),
                                             self.op_transfer_polynomial_end_travel)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("End Elevation angle"),
                                             self.op_transfer_polynomial_end_elevation)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Max. derivative order d of Travel angle"),
                                             self.op_transfer_polynomial_max_derivative_order_travel)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Max. derivative order d of Elevation angle"),
                                             self.op_transfer_polynomial_max_derivative_order_elevation)
        op_transfer_polynomial_layout.addRow(QtWidgets.QLabel("Use Prototype Function"),
                                             self.op_transfer_polynomial_prototype_checkbox)
        self.trajectory_frame_stack.addWidget(op_transfer_polynomial_frame)

        # Operating Point Transfer Gevrey
        self.trajectory_combo.insertItem(2, "Operating Point Transfer (Gevrey)")
        op_transfer_gevrey_frame = QtWidgets.QFrame()
        op_transfer_gevrey_layout = QtWidgets.QFormLayout()
        op_transfer_gevrey_frame.setLayout(op_transfer_gevrey_layout)

        self.op_transfer_gevrey_t0 = QtWidgets.QDoubleSpinBox()
        self.op_transfer_gevrey_t0.setValue(0)
        self.op_transfer_gevrey_tf = QtWidgets.QDoubleSpinBox()
        self.op_transfer_gevrey_tf.setValue(2)
        self.op_transfer_gevrey_start_travel = QtWidgets.QDoubleSpinBox()
        self.op_transfer_gevrey_start_elevation = QtWidgets.QDoubleSpinBox()
        self.op_transfer_gevrey_end_travel = QtWidgets.QDoubleSpinBox()
        self.op_transfer_gevrey_end_travel.setValue(45)
        self.op_transfer_gevrey_end_elevation = QtWidgets.QDoubleSpinBox()
        self.op_transfer_gevrey_end_elevation.setValue(10)
        self.op_transfer_gevrey_sigma = QtWidgets.QLineEdit("3")
        self.op_transfer_gevrey_sigma.setValidator(QtGui.QDoubleValidator())
        self.op_transfer_gevrey_max_derivative_order_travel = QtWidgets.QLineEdit("4")
        self.op_transfer_gevrey_max_derivative_order_travel.setValidator(QtGui.QIntValidator(0, 999999))
        self.op_transfer_gevrey_max_derivative_order_elevation = QtWidgets.QLineEdit("4")
        self.op_transfer_gevrey_max_derivative_order_elevation.setValidator(QtGui.QIntValidator(0, 999999))
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("Initial Time t0"),
                                         self.op_transfer_gevrey_t0)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("Final Time tf"),
                                         self.op_transfer_gevrey_tf)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("Start Travel angle"),
                                             self.op_transfer_gevrey_start_travel)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("Start Elevation Angle"),
                                             self.op_transfer_gevrey_start_elevation)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("End Travel angle"),
                                             self.op_transfer_gevrey_end_travel)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("End Elevation angle"),
                                             self.op_transfer_gevrey_end_elevation)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("Sigma"),
                                                    self.op_transfer_gevrey_sigma)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("Max. derivative order d of Travel angle"),
                                             self.op_transfer_gevrey_max_derivative_order_travel)
        op_transfer_gevrey_layout.addRow(QtWidgets.QLabel("Max. derivative order d of Elevation angle"),
                                             self.op_transfer_gevrey_max_derivative_order_elevation)
        self.trajectory_frame_stack.addWidget(op_transfer_gevrey_frame)

        # Constant operating point
        self.trajectory_combo.insertItem(3, "Constant Operating Point")
        constant_op_frame = QtWidgets.QFrame()
        constant_op_layout = QtWidgets.QFormLayout()
        constant_op_frame.setLayout(constant_op_layout)

        self.constant_op_travel = QtWidgets.QDoubleSpinBox()
        self.constant_op_elevation = QtWidgets.QDoubleSpinBox()

        constant_op_layout.addRow(QtWidgets.QLabel("OP Travel"), self.constant_op_travel)
        constant_op_layout.addRow(QtWidgets.QLabel("OP Elevation"), self.constant_op_elevation)

        self.trajectory_frame_stack.addWidget(constant_op_frame)

        # Wave
        self.trajectory_combo.insertItem(4, "Wave")
        wave_frame = QtWidgets.QFrame()
        wave_layout = QtWidgets.QFormLayout()
        wave_frame.setLayout(wave_layout)

        self.wave_amplitude = QtWidgets.QDoubleSpinBox()
        self.wave_amplitude.setValue(20)
        self.wave_period_t = QtWidgets.QDoubleSpinBox()
        self.wave_period_t.setValue(8)
        self.wave_period_lambda = QtWidgets.QDoubleSpinBox()
        self.wave_period_lambda.setValue(40)

        wave_layout.addRow(QtWidgets.QLabel("Amplitude"), self.wave_amplitude)
        wave_layout.addRow(QtWidgets.QLabel("Time period"), self.wave_period_t)
        wave_layout.addRow(QtWidgets.QLabel("Travel period"), self.wave_period_lambda)

        self.trajectory_frame_stack.addWidget(wave_frame)

        # Finishing touches
        scroll_area.setWidget(self.trajectory_frame_stack)
        main_layout.addWidget(scroll_area, 1)
        main_layout.addWidget(self.trajectory_combo)
        self.trajectory_combo.currentIndexChanged.connect(self.on_trajectory_combo_select)
        self.trajectory_combo.setCurrentIndex(0)

    def on_trajectory_combo_select(self):
        # print("combo select: " + str(self.trajectory_combo.currentIndex()))
        # print("count = " + str(self.trajectory_frame_stack.count()))
        self.trajectory_frame_stack.setCurrentIndex(self.trajectory_combo.currentIndex())

    # def get_operating_point(self):
    #     """
    #     :return: travel (rad), elevation (rad)
    #     """
    #     return self.fixed_op_travel.value() / 180.0 * np.pi, self.fixed_op_elevation.value() / 180.0 * np.pi

    def get_planner(self):
        """
        :return: planner object for obtaining trajectories
        """
        combo_idx = self.trajectory_combo.currentIndex()
        # the chosen combo entry defines the type of planner that is returned
        if combo_idx == 0:
            print("polynomial")
            # Polynomial Planner was selected
            t0 = self.op_transfer_polynomial_t0.value()
            tf = self.op_transfer_polynomial_tf.value()
            travel_start = self.op_transfer_polynomial_start_travel.value()
            elevation_start = self.op_transfer_polynomial_start_elevation.value()
            travel_end = self.op_transfer_polynomial_end_travel.value()
            elevation_end = self.op_transfer_polynomial_end_elevation.value()
            d_travel = int(self.op_transfer_polynomial_max_derivative_order_travel.text())
            d_elevation = int(self.op_transfer_polynomial_max_derivative_order_elevation.text())
            boolPrototype = self.op_transfer_polynomial_prototype_checkbox.checkState() == 2
            # because it is an operation point transfer all derivatives are supposed to be zero in the ending
            # and at the start
            YA_travel = np.zeros(d_travel+1)
            YB_travel = np.zeros(d_travel+1)
            YA_travel[0] = travel_start
            YB_travel[0] = travel_end
            YA_elevation = np.zeros(d_elevation+1)
            YB_elevation = np.zeros(d_elevation+1)
            YA_elevation[0] = elevation_start
            YB_elevation[0] = elevation_end
            if boolPrototype:
                print("prototype")
                planner_travel = Planner.PrototypePlanner(YA_travel, YB_travel, t0, tf, d_travel)
                planner_elevation = Planner.PrototypePlanner(YA_elevation, YB_elevation, t0, tf, d_elevation)
            else:
                print("simple polynom")
                planner_travel = Planner.PolynomialPlanner(YA_travel, YB_travel, t0, tf, d_travel)
                planner_elevation = Planner.PolynomialPlanner(YA_elevation, YB_elevation, t0, tf, d_elevation)
        elif combo_idx == 1:
            # Gevrey Planner is selected
            print("gevrey")
            t0 = self.op_transfer_gevrey_t0.value()
            tf = self.op_transfer_gevrey_tf.value()
            travel_start = self.op_transfer_gevrey_start_travel.value()
            elevation_start = self.op_transfer_gevrey_start_elevation.value()
            travel_end = self.op_transfer_gevrey_end_travel.value()
            elevation_end = self.op_transfer_gevrey_end_elevation.value()
            sigma = float(self.op_transfer_gevrey_sigma.text())
            d_travel = int(self.op_transfer_gevrey_max_derivative_order_travel.text())
            d_elevation = int(self.op_transfer_gevrey_max_derivative_order_elevation.text())
            # because it is an operation point transfer all derivatives are supposed to be zero in the ending
            # and at the start
            YA_travel = np.zeros(d_travel + 1)
            YB_travel = np.zeros(d_travel + 1)
            YA_travel[0] = travel_start
            YB_travel[0] = travel_end
            YA_elevation = np.zeros(d_elevation + 1)
            YB_elevation = np.zeros(d_elevation + 1)
            YA_elevation[0] = elevation_start
            YB_elevation[0] = elevation_end
            planner_travel = Planner.GevreyPlanner(YA_travel, YB_travel, t0, tf, d_travel, sigma)
            planner_elevation = Planner.GevreyPlanner(YA_elevation, YB_elevation, t0, tf, d_elevation, sigma)
        elif combo_idx == 2:
            # Constant OP is selected
            op_travel = self.constant_op_travel.value()
            op_elevation = self.constant_op_elevation.value()
            planner_travel = Planner.ConstantTrajectory(op_travel)
            planner_elevation = Planner.ConstantTrajectory(op_elevation)
        elif combo_idx == 3:
            # Wave trajectory is selected
            wave_amplitude = self.wave_amplitude.value()
            wave_period_t = self.wave_period_t.value()
            wave_period_lambda = self.wave_period_lambda.value()

            planner_travel = Planner.WaveTrajectory(Planner.WaveTrajectory.TrajectoryComponent.TRAVEL,
                                                    wave_amplitude, wave_period_t, wave_period_lambda)
            planner_elevation = Planner.WaveTrajectory(Planner.WaveTrajectory.TrajectoryComponent.ELEVATION,
                                                    wave_amplitude, wave_period_t, wave_period_lambda)
        else:
            print("[ERROR] Combox index was not recognized at generating trajectories")

        return planner_travel, planner_elevation
