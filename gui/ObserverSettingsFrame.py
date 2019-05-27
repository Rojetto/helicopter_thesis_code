from PyQt5 import QtWidgets
import numpy as np
from observer import Observer
from ModelConstants import ModelType


class ObserverSettingsFrame(QtWidgets.QFrame):
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
        # self.linear_kalman_lamb_op = QtWidgets.QDoubleSpinBox()
        # self.linear_kalman_lamb_op.setMinimum(-9999)
        self.linear_kalman_e_op = QtWidgets.QDoubleSpinBox()
        self.linear_kalman_e_op.setMinimum(-9999)

        # linear_kalman_layout.addRow(QtWidgets.QLabel("Lambda operational point in degree"), self.linear_kalman_lamb_op)
        linear_kalman_layout.addRow(QtWidgets.QLabel("e operational point in degree"), self.linear_kalman_e_op)

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

        self.linear_kalman_init_p.setDisabled(True)
        self.linear_kalman_init_e.setDisabled(True)
        self.linear_kalman_init_lamb.setDisabled(True)
        linear_kalman_layout.addRow(QtWidgets.QLabel("Initial state (p,e,lamb,dp(°/s),de(°/s),dlamb(°/s),f(1/s),b(1/s))"), linear_kalman_init_layout)

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

        linear_kalman_layout.addRow(QtWidgets.QLabel("Initial Cov = diag(p(°^2),e(°^2), lamb(°^2), dp(°^2/s),de(°^2/s), dlamb(°^2/s),f,b)"),
                                    linear_kalman_init_cov_layout)

        # 4. Model type
        self.linear_kalman_model_type = QtWidgets.QComboBox()
        self.linear_kalman_model_type.addItems(["simple", "gyromoment"])
        linear_kalman_layout.addRow(QtWidgets.QLabel("Model type:"), self.linear_kalman_model_type)

        # 5. Number of Outputs
        self.linear_kalman_nOutputs_combo = QtWidgets.QComboBox()
        self.linear_kalman_nOutputs_combo.addItems(["p, e, lambda", "p, e, lambda, f, b"])
        linear_kalman_layout.addRow(QtWidgets.QLabel("Measured variables: "), self.linear_kalman_nOutputs_combo)
        self.linear_kalman_model_type.currentIndexChanged.connect(self.on_linear_model_changed)
        self.linear_kalman_nOutputs_combo.setDisabled(True)

        # 6. Should set limit or not
        self.linear_kalman_should_limit_checkbox = QtWidgets.QCheckBox("Limit angles")
        linear_kalman_layout.addRow(self.linear_kalman_should_limit_checkbox, QtWidgets.QLabel("If this is checked, the angle values are limited."))

        # 7. Dynamic Inertia
        self.linear_kalman_dynamic_inertia_checkbox = QtWidgets.QCheckBox("Dynamic Inertia")
        linear_kalman_layout.addRow(self.linear_kalman_dynamic_inertia_checkbox,
                                    QtWidgets.QLabel("If this is checked, the model includes dynamic inertia."))
        self.linear_kalman_dynamic_inertia_checkbox.setDisabled(True)

        self.observer_frame_stack.addWidget(linear_kalman_frame)

        # Extended Kalman Filter
        self.observer_combo.insertItem(2, "Extended Kalman Filter")
        ext_kalman_frame = QtWidgets.QFrame()
        ext_kalman_layout = QtWidgets.QFormLayout()
        ext_kalman_frame.setLayout(ext_kalman_layout)

        # 1. initial value
        self.ext_kalman_init_p = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_e = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_lamb = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_dp = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_de = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_dlamb = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_f = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_b = QtWidgets.QDoubleSpinBox()

        ext_kalman_init_layout = QtWidgets.QHBoxLayout()
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_p)
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_e)
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_lamb)
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_dp)
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_de)
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_dlamb)
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_f)
        ext_kalman_init_layout.addWidget(self.ext_kalman_init_b)

        self.ext_kalman_init_p.setDisabled(True)
        self.ext_kalman_init_e.setDisabled(True)
        self.ext_kalman_init_lamb.setDisabled(True)
        ext_kalman_layout.addRow(QtWidgets.QLabel("Initial state (p,e,lamb,dp(°/s),de(°/s),dlamb(°/s),f(1/s),b(1/s))"),
                                    ext_kalman_init_layout)

        # 2. initial covariance matrix
        self.ext_kalman_init_cov_p = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_cov_e = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_cov_lamb = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_cov_dp = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_cov_de = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_cov_dlamb = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_cov_f = QtWidgets.QDoubleSpinBox()
        self.ext_kalman_init_cov_b = QtWidgets.QDoubleSpinBox()

        ext_kalman_init_cov_layout = QtWidgets.QHBoxLayout()

        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_p)
        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_e)
        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_lamb)
        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_dp)
        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_de)
        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_dlamb)
        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_f)
        ext_kalman_init_cov_layout.addWidget(self.ext_kalman_init_cov_b)

        ext_kalman_layout.addRow(QtWidgets.QLabel("Initial Cov = diag(p(°^2),e(°^2), lamb(°^2), dp(°^2/s),de(°^2/s), dlamb(°^2/s),f,b)"),
                                    ext_kalman_init_cov_layout)

        # 3. Model type
        self.ext_kalman_model_type = QtWidgets.QComboBox()
        self.ext_kalman_model_type.addItems(["simple", "gyromoment"])
        ext_kalman_layout.addRow(QtWidgets.QLabel("Model type:"), self.ext_kalman_model_type)

        # 4. Number of Outputs
        self.ext_kalman_nOutputs_combo = QtWidgets.QComboBox()
        self.ext_kalman_nOutputs_combo.addItems(["p, e, lambda", "p, e, lambda, f, b"])
        ext_kalman_layout.addRow(QtWidgets.QLabel("Measured variables: "), self.ext_kalman_nOutputs_combo)
        self.ext_kalman_model_type.currentIndexChanged.connect(self.on_ext_model_changed)
        self.ext_kalman_nOutputs_combo.setDisabled(True)

        # 6. Should set limit or not
        self.ext_kalman_should_limit_checkbox = QtWidgets.QCheckBox("Limit angles")
        ext_kalman_layout.addRow(self.ext_kalman_should_limit_checkbox,
                                    QtWidgets.QLabel("If this is checked, the angle values are limited."))

        # 7. Dynamic Inertia
        self.ext_kalman_dynamic_inertia_checkbox = QtWidgets.QCheckBox("Dynamic Inertia")
        ext_kalman_layout.addRow(self.ext_kalman_dynamic_inertia_checkbox,
                                    QtWidgets.QLabel("If this is checked, the model includes dynamic inertia."))
        self.ext_kalman_dynamic_inertia_checkbox.setDisabled(True)

        self.ext_kalman_model_type.setCurrentIndex(1)
        self.ext_kalman_dynamic_inertia_checkbox.setChecked(True)
        self.ext_kalman_should_limit_checkbox.setChecked(True)

        self.observer_frame_stack.addWidget(ext_kalman_frame)


        # No Kalman Filter
        self.observer_combo.insertItem(3, "No Kalman Filter")
        no_kalman_frame = QtWidgets.QFrame()
        no_kalman_layout = QtWidgets.QFormLayout()
        no_kalman_frame.setLayout(no_kalman_layout)

        self.observer_frame_stack.addWidget(no_kalman_frame)

        # # Test Kalman Filter
        # self.observer_combo.insertItem(2, "Test Kalman Filter")
        # test_kalman_frame = QtWidgets.QFrame()
        # test_kalman_layout = QtWidgets.QFormLayout()
        # test_kalman_frame.setLayout(test_kalman_layout)
        #
        # self.observer_frame_stack.addWidget(test_kalman_frame)
        #
        # # Extended Kalman Filter for easy model
        # self.observer_combo.insertItem(3, "Extended Kalman Filter for Simple Model")
        # ext_easy_kalman_frame = QtWidgets.QFrame()
        # ext_easy_kalman_layout = QtWidgets.QFormLayout()
        # ext_easy_kalman_frame.setLayout(ext_easy_kalman_layout)
        #
        # self.observer_frame_stack.addWidget(ext_easy_kalman_frame)

        # # Ext. Kalman Filter for Gyromoment model
        # self.observer_combo.insertItem(5, "Extended Kalman Filter for Gyromoment model (5-element-y)")
        # ext_gyro_kalman_frame = QtWidgets.QFrame()
        # ext_gyro_kalman_layout = QtWidgets.QFormLayout()
        # ext_gyro_kalman_frame.setLayout(ext_gyro_kalman_layout)
        #
        # self.observer_frame_stack.addWidget(ext_gyro_kalman_frame)
        #
        # # Ext. Kalman Filter for Gyromoment model
        # self.observer_combo.insertItem(6, "Extended Kalman Filter for Gyromoment model (3-element-y)")
        # ext_gyro_3_kalman_frame = QtWidgets.QFrame()
        # ext_gyro_3_kalman_layout = QtWidgets.QFormLayout()
        # ext_gyro_3_kalman_frame.setLayout(ext_gyro_3_kalman_layout)
        #
        # self.observer_frame_stack.addWidget(ext_gyro_3_kalman_frame)



        # Finishing touches
        scroll_area.setWidget(self.observer_frame_stack)
        main_layout.addWidget(scroll_area, 1)
        main_layout.addWidget(self.observer_combo)
        self.observer_combo.currentIndexChanged.connect(self.on_observer_combo_select)
        self.observer_combo.setCurrentIndex(2)

    def on_observer_combo_select(self):
        self.observer_frame_stack.setCurrentIndex(self.observer_combo.currentIndex())

    def on_linear_model_changed(self):
        model_name = self.linear_kalman_model_type.currentText()
        if model_name == "simple":
            self.linear_kalman_nOutputs_combo.setCurrentIndex(0)
            self.linear_kalman_nOutputs_combo.setDisabled(True)
            self.linear_kalman_dynamic_inertia_checkbox.setChecked(False)
            self.linear_kalman_dynamic_inertia_checkbox.setDisabled(True)
        elif model_name == "gyromoment":
            self.linear_kalman_nOutputs_combo.setCurrentIndex(1)
            self.linear_kalman_nOutputs_combo.setDisabled(True)
            self.linear_kalman_dynamic_inertia_checkbox.setDisabled(False)

        else:
            raise NotImplementedError("Combo element not implemented yet")

    def on_ext_model_changed(self):
        model_name = self.ext_kalman_model_type.currentText()
        if model_name == "simple":
            self.ext_kalman_nOutputs_combo.setCurrentIndex(0)
            self.ext_kalman_nOutputs_combo.setDisabled(True)
            self.ext_kalman_dynamic_inertia_checkbox.setChecked(False)
            self.ext_kalman_dynamic_inertia_checkbox.setDisabled(True)
        elif model_name == "gyromoment":
            self.ext_kalman_nOutputs_combo.setCurrentIndex(1)
            self.ext_kalman_nOutputs_combo.setDisabled(False)
            self.ext_kalman_dynamic_inertia_checkbox.setDisabled(False)
        else:
            raise NotImplementedError("Combo element not implemented yet")

    def get_observer(self, stepSize, noise_settings):
        """:arg timeStep: step size of simulation
        :return observer object """
        input_matrix_variance, output_matrix_variance, input_noise_variance, output_noise_variance, bdisable_input_noise, bdisable_output_noise, bdisable_N_matrix, bdisable_W_matrix = noise_settings
        combo_idx = self.observer_combo.currentIndex()
        # the chosen combo entry defines the type of planner that is returned
        if combo_idx == 0:
            print("Linear Kalman Filter")
            # 1. Operational Point
            # lamb_op = self.linear_kalman_lamb_op.value() * np.pi / 180
            e_op = self.linear_kalman_e_op.value() * np.pi / 180

            # 2. Initial values
            p_init = self.linear_kalman_init_p.value() * np.pi / 180
            e_init = self.linear_kalman_init_e.value() * np.pi / 180
            lamb_init = self.linear_kalman_init_lamb.value() * np.pi / 180
            dp_init = self.linear_kalman_init_dp.value() * np.pi / 180
            de_init = self.linear_kalman_init_de.value() * np.pi / 180
            dlamb_init = self.linear_kalman_init_dlamb.value() * np.pi / 180
            f_init = self.linear_kalman_init_f.value()
            b_init = self.linear_kalman_init_b.value()

            # 3. Initial covariance
            p_cov_init = self.linear_kalman_init_cov_p.value() * (np.pi / 180)**2
            e_cov_init = self.linear_kalman_init_cov_e.value() * (np.pi / 180)**2
            lamb_cov_init = self.linear_kalman_init_cov_lamb.value() * (np.pi / 180)**2
            dp_cov_init = self.linear_kalman_init_cov_dp.value() * (np.pi / 180)**2
            de_cov_init = self.linear_kalman_init_cov_de.value() * (np.pi / 180)**2
            dlamb_cov_init = self.linear_kalman_init_cov_dlamb.value() * (np.pi / 180)**2
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

            # 6. should check limits
            should_check_limits = self.linear_kalman_should_limit_checkbox.checkState() == 2

            # 7. dynamic inertia
            dynamic_inertia = self.linear_kalman_dynamic_inertia_checkbox.checkState() == 2


            if model_type == ModelType.GYROMOMENT:
                init_state_vector = np.array([p_init, e_init, lamb_init, dp_init, de_init, dlamb_init, f_init, b_init])
                init_cov_matrix = np.diagflat([p_cov_init, e_cov_init, lamb_cov_init,
                                               dp_cov_init, de_cov_init, dlamb_cov_init,
                                               f_cov_init, b_cov_init])
            else:
                init_state_vector = np.array([p_init, e_init, lamb_init, dp_init, de_init, dlamb_init])
                init_cov_matrix = np.diagflat([p_cov_init, e_cov_init, lamb_cov_init,
                                               dp_cov_init, de_cov_init, dlamb_cov_init])

            observer = Observer.LinearKalmanFilterWithLimits(init_state_vector, init_cov_matrix, model_type, e_op,
                                                             nOutputs, stepSize, input_matrix_variance, output_matrix_variance,
                                                             input_noise_variance, output_noise_variance)

            observer.set_should_limit(should_check_limits)
            observer.set_dynamic_inertia(dynamic_inertia)

        elif combo_idx == 1:
            print("Extended Kalman Filter")
            # 2. Initial values
            p_init = self.ext_kalman_init_p.value() * np.pi / 180
            e_init = self.ext_kalman_init_e.value() * np.pi / 180
            lamb_init = self.ext_kalman_init_lamb.value() * np.pi / 180
            dp_init = self.ext_kalman_init_dp.value() * np.pi / 180
            de_init = self.ext_kalman_init_de.value() * np.pi / 180
            dlamb_init = self.ext_kalman_init_dlamb.value() * np.pi / 180
            f_init = self.ext_kalman_init_f.value()
            b_init = self.ext_kalman_init_b.value()

            # 3. Initial covariance
            p_cov_init = self.ext_kalman_init_cov_p.value() * (np.pi / 180)**2
            e_cov_init = self.ext_kalman_init_cov_e.value() * (np.pi / 180)**2
            lamb_cov_init = self.ext_kalman_init_cov_lamb.value() * (np.pi / 180)**2
            dp_cov_init = self.ext_kalman_init_cov_dp.value() * (np.pi / 180)**2
            de_cov_init = self.ext_kalman_init_cov_de.value() * (np.pi / 180)**2
            dlamb_cov_init = self.ext_kalman_init_cov_dlamb.value() * (np.pi / 180)**2
            f_cov_init = self.ext_kalman_init_cov_f.value()
            b_cov_init = self.ext_kalman_init_cov_b.value()

            # 4. Model Type
            model_name = self.ext_kalman_model_type.currentText()
            if model_name == "simple":
                model_type = ModelType.EASY
            elif model_name == "gyromoment":
                model_type = ModelType.GYROMOMENT
            else:
                raise NotImplementedError("Combo element not implemented yet")

            # 5. Number of Outputs
            nOutputs_text = self.ext_kalman_nOutputs_combo.currentText()
            if nOutputs_text == "p, e, lambda":
                nOutputs = 3
            elif nOutputs_text == "p, e, lambda, f, b":
                nOutputs = 5
            else:
                raise NotImplementedError("Combo element not implemented yet")

            # 6. should check limits
            should_check_limits = self.ext_kalman_should_limit_checkbox.checkState() == 2

            # 7. dynamic inertia
            dynamic_inertia = self.ext_kalman_dynamic_inertia_checkbox.checkState() == 2

            init_state_vector = np.array([p_init, e_init, lamb_init, dp_init, de_init, dlamb_init, f_init, b_init])
            init_cov_matrix = np.diagflat([p_cov_init, e_cov_init, lamb_cov_init,
                                           dp_cov_init, de_cov_init, dlamb_cov_init,
                                           f_cov_init, b_cov_init])

            if model_type == ModelType.GYROMOMENT:
                if nOutputs == 5:
                    observer = Observer.ExtKalmanFilterGyroModelLimits(init_state_vector, init_cov_matrix,
                                                                       stepSize, input_matrix_variance,
                                                                       output_matrix_variance,
                                                                       input_noise_variance, output_noise_variance)
                elif nOutputs == 3:
                    observer = Observer.ExtKalmanFilterGyroModelLimitsOnly3(init_state_vector, init_cov_matrix,
                                                                            stepSize, input_matrix_variance,
                                                                            output_matrix_variance,
                                                                            input_noise_variance, output_noise_variance)
            else:
                observer = Observer.ExtKalmanFilterEasyModelLimits(init_state_vector, init_cov_matrix,
                                                                   stepSize, input_matrix_variance,
                                                                   output_matrix_variance,
                                                                   input_noise_variance, output_noise_variance)

            observer.set_should_limit(should_check_limits)
            observer.set_dynamic_inertia(dynamic_inertia)

        elif combo_idx == 2:
            print("No Kalman filter")
            observer = Observer.NoKalmanFilter(np.zeros(6), np.diag(np.zeros(6)))
        else:
            raise NotImplementedError("This option of the combo box is not implemented yet.")

        # disable noise or enable it
        if bdisable_input_noise:
            print("Disable input noise")
            observer.toggle_input_noise(False)

        if bdisable_output_noise:
            print("Disable output noise")
            observer.toggle_output_noise(False)

        if bdisable_N_matrix:
            print("Disable N-Matrix")
            observer.toggle_input_variance(False)

        if bdisable_W_matrix:
            print("Disable W-Matrix")
            observer.toggle_output_variance(False)

        return observer

