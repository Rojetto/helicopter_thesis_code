from PyQt5 import QtWidgets, QtGui
import numpy as np
import Planner
from Disturbance import Disturbance, DisturbanceStep, DisturbanceSinus, DisturbanceRect, NoDisturbance
from ModelConstants import ObserverType
import Observer


class ObserverFrame(QtWidgets.QFrame):
    def __init__(self):
        # QtWidgets.QFrame.__init__(self)
        super().__init__()

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setFixedHeight(200)

        self.observer_combo = QtWidgets.QComboBox()
        self.observer_frame_stack = QtWidgets.QStackedWidget()

        # fill the QStackWidget manually with Frames. If this becomes too tedious and confusing
        # then maybe switch to doing something with more abstraction

        # Linear Kalman Filter
        self.observer_combo.insertItem(1, "Linear Kalman Filter")
        linear_kalman_frame = QtWidgets.QFrame()
        linear_kalman_layout = QtWidgets.QFormLayout()
        linear_kalman_frame.setLayout(linear_kalman_layout)

        # 1. Operational Point
        self.linear_kalman_lamb_op = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_lamb_op.setMinimum(-9999)
        self.linear_kalman_e_op = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_e_op.setMinimum(-9999)

        linear_kalman_layout.addRow(QtWidgets.QLabel("Lambda operational point"), self.linear_kalman_lamb_op)
        linear_kalman_layout.addRow(QtWidgets.QLabel("e operational point"), self.linear_kalman_e_op)

        # 2. initial value
        self.linear_kalman_init_p = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_e = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_lamb = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_dp = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_de = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_dlamb = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_f = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_b = QtWidgets.QDoubleSpinBox()

        linear_kalman_init_layout = QtWidgets.QHBoxLayout()
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_p)
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_e)
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_lamb)
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_dp)
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_de)
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_dlamb)
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_f)
        linear_kalman_init_layout.addWidget(self.linear_kalman_init_b)

        linear_kalman_layout.addRow(QtWidgets.QLabel("Initial state (p,e,lamb,dp,de,dlamb,f,b)"), linear_kalman_init_layout)

        # 3. initial covariance matrix
        self.linear_kalman_init_cov_p = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_cov_e = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_cov_lamb = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_cov_dp = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_cov_de = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_cov_dlamb = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_cov_f = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_init_cov_b = QtWidgets.QDoubleSpinBox()

        linear_kalman_init_cov_layout = QtWidgets.QHBoxLayout()

        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_p)
        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_e)
        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_lamb)
        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_dp)
        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_de)
        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_dlamb)
        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_f)
        linear_kalman_init_cov_layout.addWidget(self.linear_kalman_init_cov_b)

        linear_kalman_layout.addRow(QtWidgets.QLabel("Initial Cov = diag(p,e, lamb, dp,de, dlamb,f,b)"),
                                    linear_kalman_init_cov_layout)


        self.observer_frame_stack.addWidget(linear_kalman_frame)

        # Test Kalman Filter
        self.observer_combo.insertItem(2, "Test Kalman Filter")
        test_kalman_frame = QtWidgets.QFrame()
        test_kalman_layout = QtWidgets.QFormLayout()
        test_kalman_frame.setLayout(test_kalman_layout)

        self.observer_frame_stack.addWidget(test_kalman_frame)

        # Extended Kalman Filter for easy model
        self.observer_combo.insertItem(3, "Extended Kalman Filter for Easy Model")
        ext_easy_kalman_frame = QtWidgets.QFrame()
        ext_easy_kalman_layout = QtWidgets.QFormLayout()
        ext_easy_kalman_frame.setLayout(ext_easy_kalman_layout)

        self.observer_frame_stack.addWidget(ext_easy_kalman_frame)

        # No Kalman Filter
        self.observer_combo.insertItem(4, "No Kalman Filter")
        no_kalman_frame = QtWidgets.QFrame()
        no_kalman_layout = QtWidgets.QFormLayout()
        no_kalman_frame.setLayout(no_kalman_layout)

        self.observer_frame_stack.addWidget(no_kalman_frame)



        # Finishing touches
        scroll_area.setWidget(self.observer_frame_stack)
        main_layout.addWidget(scroll_area, 1)
        main_layout.addWidget(self.observer_combo)
        self.observer_combo.currentIndexChanged.connect(self.on_observer_combo_select)
        self.observer_combo.setCurrentIndex(2)

    def on_observer_combo_select(self):
        self.observer_frame_stack.setCurrentIndex(self.observer_combo.currentIndex())

    def get_observer(self):
        """:return observer object """
        combo_idx = self.observer_combo.currentIndex()
        # the chosen combo entry defines the type of planner that is returned
        if combo_idx == 0:
            print("Linear Kalman Filter")
            # 1. Operational Point
            lamb_op = self.linear_kalman_init_p.value()
            e_op = self.linear_kalman_init_e.value()

            # 2. Initial values
            p_init = self.linear_kalman_init_p.value()
            e_init = self.linear_kalman_init_e.value()
            lamb_init = self.linear_kalman_init_lamb.value()
            dp_init = self.linear_kalman_init_dp.value()
            de_init = self.linear_kalman_init_de.value()
            dlamb_init = self.linear_kalman_init_dlamb.value()

            # 3. Initial covariance
            p_cov_init = self.linear_kalman_init_cov_p.value()
            e_cov_init = self.linear_kalman_init_cov_e.value()
            lamb_cov_init = self.linear_kalman_init_cov_lamb.value()
            dp_cov_init = self.linear_kalman_init_cov_dp.value()
            de_cov_init = self.linear_kalman_init_cov_de.value()
            dlamb_cov_init = self.linear_kalman_init_cov_dlamb.value()

            operating_point = np.array([0, e_op, lamb_op])
            init_state_vector = np.array([p_init, e_init, lamb_init, dp_init, de_init, dlamb_init])
            init_cov_matrix = np.diagflat([p_cov_init, e_cov_init, lamb_cov_init,
                                           dp_cov_init, de_cov_init, dlamb_cov_init])

            observer = Observer.LinearKalmanFilter(init_state_vector, init_cov_matrix, operating_point)
        elif combo_idx == 1:
            print("TestKalmanFilter")
            observer = Observer.TestKalmanFilter([0, 0, 0, 0, 0, 0], np.diag([0, 0, 0, 0, 0, 0]))
        elif combo_idx == 2:
            print("Ext. Kalman Filter for Easy Model")
            observer = Observer.ExtKalmanFilterEasyModel([0, 0, 0, 0, 0, 0], np.diag([0, 0, 0, 0, 0, 0]))
        elif combo_idx == 3:
            print("No Kalman filter")
            observer = Observer.NoKalmanFilter(np.zeros(6), np.diag(np.zeros(6)))
        else:
            raise NotImplementedError("This option of the combo box is not yet implemented.")

        return observer

