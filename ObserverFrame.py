from PyQt5 import QtWidgets, QtGui
import numpy as np
import Planner
from Disturbance import Disturbance, DisturbanceStep, DisturbanceSinus, DisturbanceRect, NoDisturbance
from ModelConstants import ObserverType
import Observer
from ModelConstants import ModelType


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

        # 4. Model type
        self.linear_kalman_model_type = QtWidgets.QComboBox()
        self.linear_kalman_model_type.addItems(["simple", "gyromoment"])
        linear_kalman_layout.addRow(QtWidgets.QLabel("Model type:"), self.linear_kalman_model_type)

        # 5. Number of Outputs
        self.linear_kalman_nOutputs_combo = QtWidgets.QComboBox()
        self.linear_kalman_nOutputs_combo.addItems(["p, e, lambda", "p, e, lambda, f, b"])
        linear_kalman_layout.addRow(QtWidgets.QLabel("Measured variables: "), self.linear_kalman_nOutputs_combo)

        self.observer_frame_stack.addWidget(linear_kalman_frame)

        # Test Kalman Filter
        self.observer_combo.insertItem(2, "Test Kalman Filter")
        test_kalman_frame = QtWidgets.QFrame()
        test_kalman_layout = QtWidgets.QFormLayout()
        test_kalman_frame.setLayout(test_kalman_layout)

        self.observer_frame_stack.addWidget(test_kalman_frame)

        # Extended Kalman Filter for easy model
        self.observer_combo.insertItem(3, "Extended Kalman Filter for Simple Model")
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

        # Ext. Kalman Filter for Gyromoment model
        self.observer_combo.insertItem(5, "Extended Kalman Filter for Gyromoment model (5-element-y)")
        ext_gyro_kalman_frame = QtWidgets.QFrame()
        ext_gyro_kalman_layout = QtWidgets.QFormLayout()
        ext_gyro_kalman_frame.setLayout(ext_gyro_kalman_layout)

        self.observer_frame_stack.addWidget(ext_gyro_kalman_frame)

        # Ext. Kalman Filter for Gyromoment model
        self.observer_combo.insertItem(6, "Extended Kalman Filter for Gyromoment model (3-element-y)")
        ext_gyro_3_kalman_frame = QtWidgets.QFrame()
        ext_gyro_3_kalman_layout = QtWidgets.QFormLayout()
        ext_gyro_3_kalman_frame.setLayout(ext_gyro_3_kalman_layout)

        self.observer_frame_stack.addWidget(ext_gyro_3_kalman_frame)



        # Finishing touches
        scroll_area.setWidget(self.observer_frame_stack)
        main_layout.addWidget(scroll_area, 1)
        main_layout.addWidget(self.observer_combo)
        self.observer_combo.currentIndexChanged.connect(self.on_observer_combo_select)
        self.observer_combo.setCurrentIndex(4)

    def on_observer_combo_select(self):
        self.observer_frame_stack.setCurrentIndex(self.observer_combo.currentIndex())

    def get_observer(self, stepSize):
        """:arg timeStep: step size of simulation
        :return observer object """
        combo_idx = self.observer_combo.currentIndex()
        # the chosen combo entry defines the type of planner that is returned
        if combo_idx == 0:
            print("Linear Kalman Filter")
            # 1. Operational Point
            lamb_op = self.linear_kalman_lamb_op.value()
            e_op = self.linear_kalman_e_op.value()

            # 2. Initial values
            p_init = self.linear_kalman_init_p.value()
            e_init = self.linear_kalman_init_e.value()
            lamb_init = self.linear_kalman_init_lamb.value()
            dp_init = self.linear_kalman_init_dp.value()
            de_init = self.linear_kalman_init_de.value()
            dlamb_init = self.linear_kalman_init_dlamb.value()
            f_init = self.linear_kalman_init_f.value()
            b_init = self.linear_kalman_init_b.value()

            # 3. Initial covariance
            p_cov_init = self.linear_kalman_init_cov_p.value()
            e_cov_init = self.linear_kalman_init_cov_e.value()
            lamb_cov_init = self.linear_kalman_init_cov_lamb.value()
            dp_cov_init = self.linear_kalman_init_cov_dp.value()
            de_cov_init = self.linear_kalman_init_cov_de.value()
            dlamb_cov_init = self.linear_kalman_init_cov_dlamb.value()
            f_cov_init = self.linear_kalman_init_cov_f.value()
            b_cov_init = self.linear_kalman_init_cov_b.value()

            # 4. Model Type
            model_name = self.linear_kalman_model_type.currentText()
            if model_name == "simple":
                model_type = ModelType.EASY
            elif model_name == "gyromoment":
                model_type = ModelType.GYROMOMENT
            else:
                raise NotImplementedError("Combo element not implemented yet")

            # 5. Number of Outputs
            nOutputs_text = self.linear_kalman_nOutputs_combo.currentText()
            if nOutputs_text == "p, e, lambda":
                nOutputs = 3
            elif nOutputs_text == "p, e, lambda, f, b":
                nOutputs = 5
            else:
                raise NotImplementedError("Combo element not implemented yet")


            if model_type == ModelType.GYROMOMENT:
                init_state_vector = np.array([p_init, e_init, lamb_init, dp_init, de_init, dlamb_init, f_init, b_init])
                init_cov_matrix = np.diagflat([p_cov_init, e_cov_init, lamb_cov_init,
                                               dp_cov_init, de_cov_init, dlamb_cov_init,
                                               f_cov_init, b_cov_init])
            else:
                init_state_vector = np.array([p_init, e_init, lamb_init, dp_init, de_init, dlamb_init])
                init_cov_matrix = np.diagflat([p_cov_init, e_cov_init, lamb_cov_init,
                                               dp_cov_init, de_cov_init, dlamb_cov_init])

            observer = Observer.LinearKalmanFilter(init_state_vector, init_cov_matrix, model_type, e_op, lamb_op,
                                                   nOutputs, stepSize)

        elif combo_idx == 1:
            print("TestKalmanFilter")
            observer = Observer.TestKalmanFilter([0, 0, 0, 0, 0, 0, 0, 0], np.diag([0, 0, 0, 0, 0, 0, 0, 0]))
        elif combo_idx == 2:
            print("Ext. Kalman Filter for Easy Model")
            observer = Observer.ExtKalmanFilterEasyModel([0, 0, 0, 0, 0, 0, 0, 0], np.diag([0, 0, 0, 0, 0, 0, 0, 0]))
        elif combo_idx == 3:
            print("No Kalman filter")
            observer = Observer.NoKalmanFilter(np.zeros(6), np.diag(np.zeros(6)))
        elif combo_idx == 4:
            print("Ext. Kalman Filter for Gyromoment Model (5-element)")
            observer = Observer.ExtKalmanFilterGyroModelLimits(np.zeros(8), np.diag([0, 0, 0, 0, 0, 0, 0, 0]), stepSize)
            # ToDo: Implement GUI for changing this
            observer.set_should_limit(True)
        elif combo_idx == 5:
            print("Ext. Kalman Filter for Gyromoment Model(3-element)")
            observer = Observer.ExtKalmanFilterGyroModelOnly3(np.zeros(8), np.diag([0, 0, 0, 0, 0, 0, 0, 0]))
        else:
            raise NotImplementedError("This option of the combo box is not implemented yet.")

        return observer

