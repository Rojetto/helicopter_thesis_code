from PyQt5 import QtWidgets, QtGui, QtCore
import numpy as np
import Observer
from ModelConstants import ModelType
import InaccurateModelConstants as imc


class ObserverNoiseFrame(QtWidgets.QFrame):
    def __init__(self):
        # QtWidgets.QFrame.__init__(self)
        super().__init__()
        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setFixedHeight(200)
        scroll_content = QtWidgets.QWidget()
        scroll_area.setWidget(scroll_content)
        scroll_area.setWidgetResizable(True)
        scroll_area_layout = QtWidgets.QVBoxLayout()
        scroll_content.setLayout(scroll_area_layout)
        main_layout.addWidget(scroll_area)

        def build_slider_textedit_combo(min_value, max_value, init_value, callback):
            def slider_to_value(position):
                return position / 99.0 * (max_value - min_value) + min_value

            def value_to_slider(value):
                return int((value - min_value) / (max_value - min_value) * 99)

            h_layout = QtWidgets.QHBoxLayout()
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

        # Create first groupbox
        groupbox_kalman_matrices = QtWidgets.QGroupBox("Kalman Filter Variance Matrices")
        groupbox_kalman_layout = QtWidgets.QGridLayout()
        groupbox_kalman_matrices.setLayout(groupbox_kalman_layout)

        # set minimum and maximum values for the spinbox
        N_min = 0.001
        N_max = 0.02
        N_init = 0.005
        self.N_factor = 1000
        W_min = 0.1
        W_max = 2
        W_init = 0.25
        self.W_factor = 100

        # create slider with spinboxes
        kalman_matrices_N_layout, self.kalman_matrices_N_spinbox = build_slider_textedit_combo(N_min * self.N_factor,
                                                                   N_max * self.N_factor, N_init * self.N_factor,
                                                                   self.on_init_value_change)
        kalman_matrices_N_widget = QtWidgets.QWidget()
        kalman_matrices_N_widget.setLayout(kalman_matrices_N_layout)
        kalman_matrices_W_layout, self.kalman_matrices_W_spinbox = build_slider_textedit_combo(W_min * self.W_factor,
                                                                   W_max * self.W_factor, W_init * self.W_factor,
                                                                   self.on_init_value_change)
        kalman_matrices_W_widget = QtWidgets.QWidget()
        kalman_matrices_W_widget.setLayout(kalman_matrices_W_layout)

        groupbox_kalman_layout.addWidget(QtWidgets.QLabel("N variance in V^2 * " + str(self.N_factor)), 0, 0)
        groupbox_kalman_layout.addWidget(kalman_matrices_N_widget, 0, 1)
        groupbox_kalman_layout.addWidget(QtWidgets.QLabel("W variance in °^2 resp. (1/s)^2 * " + str(self.W_factor)),
                                         1, 0)
        groupbox_kalman_layout.addWidget(kalman_matrices_W_widget, 1, 1)

        # ToDo: Bring these checkboxes to life
        self.kalman_disable_input_checkbox = QtWidgets.QCheckBox("Set N-Matrix to 0")
        self.kalman_disable_output_checkbox = QtWidgets.QCheckBox("Set W-matrix to 0 (or at least very close to it)")
        groupbox_kalman_layout.addWidget(self.kalman_disable_input_checkbox, 2, 0)
        groupbox_kalman_layout.addWidget(self.kalman_disable_output_checkbox, 3, 0)

        scroll_area_layout.addWidget(groupbox_kalman_matrices)

        # create button for synchronizing matrices and noise
        self.sync_kalman_matrices_and_noise_button = QtWidgets.QPushButton("Set Kalman matrices to noise values")
        scroll_area_layout.addWidget(self.sync_kalman_matrices_and_noise_button)
        self.sync_kalman_matrices_and_noise_button.clicked.connect(self.on_set_matrix_values_from_noise)

        # create second group box
        groupbox_noise = QtWidgets.QGroupBox("Noise settings")
        groupbox_noise_layout = QtWidgets.QGridLayout()
        groupbox_noise.setLayout(groupbox_noise_layout)

        # create slider with spinboxes
        noise_N_layout, self.noise_N_spinbox = build_slider_textedit_combo(N_min * self.N_factor,
                                                                   N_max * self.N_factor, N_init * self.N_factor,
                                                                   self.on_init_value_change)
        noise_N_widget = QtWidgets.QWidget()
        noise_N_widget.setLayout(noise_N_layout)
        noise_W_layout, self.noise_W_spinbox = build_slider_textedit_combo(W_min * self.W_factor,
                                                                   W_max * self.W_factor, W_init * self.W_factor,
                                                                   self.on_init_value_change)
        noise_W_widget = QtWidgets.QWidget()
        noise_W_widget.setLayout(noise_W_layout)

        groupbox_noise_layout.addWidget(QtWidgets.QLabel("N variance in V^2 * " + str(self.N_factor)), 0, 0)
        groupbox_noise_layout.addWidget(noise_N_widget, 0, 1)
        groupbox_noise_layout.addWidget(QtWidgets.QLabel("W variance in °^2 resp. (1/s)^2 * " + str(self.W_factor)), 1, 0)
        groupbox_noise_layout.addWidget(noise_W_widget, 1, 1)

        self.noise_disable_input_checkbox = QtWidgets.QCheckBox("Input Noise")
        self.noise_disable_output_checkbox = QtWidgets.QCheckBox("Output Noise")
        groupbox_noise_layout.addWidget(self.noise_disable_input_checkbox, 2, 0)
        groupbox_noise_layout.addWidget(self.noise_disable_output_checkbox, 3, 0)

        scroll_area_layout.addWidget(groupbox_noise)

        groupbox_param_uncertainty = QtWidgets.QGroupBox("Parameter Uncertainty")
        groupbox_param_uncertainty_layout = QtWidgets.QGridLayout()
        groupbox_param_uncertainty.setLayout(groupbox_param_uncertainty_layout)

        self.lower_boundary_line_edit = QtWidgets.QLineEdit("80")
        self.upper_boundary_line_edit = QtWidgets.QLineEdit("120")
        self.scramble_params_button = QtWidgets.QPushButton("Scramble Parameters")
        self.scramble_params_button.clicked.connect(self.on_scramble_parameter_values)
        self.param_text_edit = QtWidgets.QTextEdit()
        self.param_text_edit.setReadOnly(True)

        groupbox_param_uncertainty_layout.addWidget(QtWidgets.QLabel("Lower Boundary in percent"), 0, 0)
        groupbox_param_uncertainty_layout.addWidget(self.lower_boundary_line_edit, 0, 1)
        groupbox_param_uncertainty_layout.addWidget(QtWidgets.QLabel("Upper Boundary in percent"), 0, 2)
        groupbox_param_uncertainty_layout.addWidget(self.upper_boundary_line_edit, 0, 3)
        groupbox_param_uncertainty_layout.addWidget(self.scramble_params_button, 1, 0)
        groupbox_param_uncertainty_layout.addWidget(QtWidgets.QLabel("Randomized Parameter Values"), 2, 0)
        groupbox_param_uncertainty_layout.addWidget(self.param_text_edit, 2, 1)

        scroll_area_layout.addWidget(groupbox_param_uncertainty)

        # initialize parameter edit
        text = ("l_p = " + str(imc.l_p) + "\nl_h = " + str(imc.l_h) + "\nl_c = " + str(imc.l_c) + "\nm_p = " +
                str(imc.m_p) + "\nm_c_max = " + str(imc.m_c_max) + "\nm_c = " + str(imc.m_c) + "\nm_m = " + str(
                    imc.m_m) +
                "\nr_m = " + str(imc.r_m) + "\nJ_m = " + str(imc.J_m) + "\nd_p = " + str(imc.d_p) + "\nd_e = " +
                str(imc.d_e) + "\nd_l = " + str(imc.d_l) + "\nT_f = " + str(imc.T_f) + "\nT_b = " + str(imc.T_b) +
                "\nK_f = " + str(imc.K_f) + "\nK_b = " + str(imc.K_b) + "\nK = " + str(imc.K) + "\nK_m = " +
                str(imc.K_m) + "\ng = " + str(imc.g))
        self.param_text_edit.setText(text)
        return

    def on_init_value_change(self):
        pass

    def on_scramble_parameter_values(self):
        lower_boundary = float(self.lower_boundary_line_edit.text()) / 100
        upper_boundary = float(self.upper_boundary_line_edit.text()) / 100
        imc.scrambleParameters(lower_boundary, upper_boundary)
        # display the new values in the text edit
        text = ("l_p = " + str(imc.l_p) + "\nl_h = " + str(imc.l_h) + "\nl_c = " + str(imc.l_c) + "\nm_p = " +
                str(imc.m_p) + "\nm_c_max = " + str(imc.m_c_max) + "\nm_c = " + str(imc.m_c) + "\nm_m = " + str(imc.m_m) +
                "\nr_m = " + str(imc.r_m) + "\nJ_m = " + str(imc.J_m) + "\nd_p = " + str(imc.d_p) + "\nd_e = " +
                str(imc.d_e) + "\nd_l = " + str(imc.d_l) + "\nT_f = " + str(imc.T_f) + "\nT_b = " + str(imc.T_b) +
                "\nK_f = " + str(imc.K_f) + "\nK_b = " + str(imc.K_b) + "\nK = " + str(imc.K) + "\nK_m = " +
                str(imc.K_m) + "\ng = " + str(imc.g))
        self.param_text_edit.setText(text)

    def get_noise_settings(self):
        ''':return input_matrix_variance, output_matrix_variance, input_noise_variance, output_noise_variance'''
        input_matrix_variance = self.kalman_matrices_N_spinbox.value() / self.N_factor
        output_matrix_variance = self.kalman_matrices_W_spinbox.value() / self.W_factor
        input_noise_variance = self.noise_N_spinbox.value() / self.N_factor
        output_noise_variance = self.noise_W_spinbox.value() / self.W_factor

        print("output_standard_deviation in degree = " + str(np.sqrt(output_noise_variance)))

        # convert to radian variance
        output_matrix_variance = output_matrix_variance * (np.pi/180)**2
        output_noise_variance = output_noise_variance * (np.pi/180)**2

        return input_matrix_variance, output_matrix_variance, input_noise_variance, output_noise_variance

    def on_set_matrix_values_from_noise(self):
        self.kalman_matrices_N_spinbox.setValue(self.noise_N_spinbox.value())
        self.kalman_matrices_W_spinbox.setValue(self.noise_W_spinbox.value())
