from PyQt5 import QtWidgets

from HeliSimulation import HeliSimulation, ModelType


class ModelFrame(QtWidgets.QFrame):

    def __init__(self, sim: HeliSimulation):
        QtWidgets.QFrame.__init__(self)

        self.sim = sim
        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        # Maybe it would be more consistent if we read the model type on "Start" instead of changing it here
        self.model_1_button = QtWidgets.QRadioButton("Simple model", self)
        self.model_1_button.toggled.connect(self.on_model_toggle)
        self.model_2_button = QtWidgets.QRadioButton("+ friction", self)
        self.model_2_button.toggled.connect(self.on_model_toggle)
        self.model_3_button = QtWidgets.QRadioButton("+ centripetal forces", self)
        self.model_3_button.toggled.connect(self.on_model_toggle)
        self.model_4_button = QtWidgets.QRadioButton("+ rotor speed", self)
        self.model_4_button.toggled.connect(self.on_model_toggle)
        self.model_5_button = QtWidgets.QRadioButton("+ gyro moment", self)
        self.model_5_button.toggled.connect(self.on_model_toggle)
        self.model_3_button.setChecked(True)

        self.check_limits_box = QtWidgets.QCheckBox("Limit angles")
        self.check_limits_box.setCheckState(2 if sim.should_check_limits else 0)
        self.check_limits_box.toggled.connect(self.on_limit_check_toggle)

        self.check_dynamic_inertia_torque = QtWidgets.QCheckBox("Dynamic Inertia Torque")
        self.check_dynamic_inertia_torque.setCheckState(2 if self.sim.dynamic_inertia_torque else 0)
        self.check_dynamic_inertia_torque.toggled.connect(self.on_dynamic_inertia_torque_toggle)

        layout_h = QtWidgets.QHBoxLayout()
        layout_h.addWidget(self.check_limits_box)
        layout_h.addWidget(self.check_dynamic_inertia_torque)

        main_layout.addWidget(self.model_1_button)
        main_layout.addWidget(self.model_2_button)
        main_layout.addWidget(self.model_3_button)
        main_layout.addWidget(self.model_4_button)
        main_layout.addWidget(self.model_5_button)
        main_layout.addLayout(layout_h)

    def on_model_toggle(self):
        if self.model_1_button.isChecked():
            self.sim.set_model_type(ModelType.EASY)
        elif self.model_2_button.isChecked():
            self.sim.set_model_type(ModelType.FRICTION)
        elif self.model_3_button.isChecked():
            self.sim.set_model_type(ModelType.CENTRIPETAL)
        elif self.model_4_button.isChecked():
            self.sim.set_model_type(ModelType.ROTORSPEED)
        elif self.model_5_button.isChecked():
            self.sim.set_model_type(ModelType.GYROMOMENT)

    def on_limit_check_toggle(self):
        self.sim.set_should_limit(self.check_limits_box.checkState() == 2)

    def on_dynamic_inertia_torque_toggle(self):
        self.sim.set_dynamic_inertia_torque(self.check_dynamic_inertia_torque.checkState() == 2)

