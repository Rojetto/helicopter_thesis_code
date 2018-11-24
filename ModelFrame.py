from PyQt5 import QtWidgets

from HeliSimulation import HeliSimulation, ModelType


class ModelFrame(QtWidgets.QFrame):

    def __init__(self, sim: HeliSimulation):
        QtWidgets.QFrame.__init__(self)

        self.sim = sim
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        # Maybe it would be more consistent if we read the model type on "Start" instead of changing it here
        self.model_1_button = QtWidgets.QRadioButton("Simple model", self)
        self.model_1_button.toggled.connect(self.on_model_toggle)
        self.model_2_button = QtWidgets.QRadioButton("+ friction", self)
        self.model_2_button.toggled.connect(self.on_model_toggle)
        self.model_3_button = QtWidgets.QRadioButton("+ centripetal forces", self)
        self.model_3_button.toggled.connect(self.on_model_toggle)
        self.model_3_button.setChecked(True)

        self.check_limits_box = QtWidgets.QCheckBox("Limit angles")
        self.check_limits_box.setCheckState(2 if sim.should_check_limits else 0)
        self.check_limits_box.toggled.connect(self.on_limit_check_toggle)

        layout.addWidget(self.model_1_button)
        layout.addWidget(self.model_2_button)
        layout.addWidget(self.model_3_button)
        layout.addWidget(self.check_limits_box)

    def on_model_toggle(self):
        if self.model_1_button.isChecked():
            self.sim.set_model_type(ModelType.EASY)
        elif self.model_2_button.isChecked():
            self.sim.set_model_type(ModelType.FRICTION)
        elif self.model_3_button.isChecked():
            self.sim.set_model_type(ModelType.CENTRIPETAL)

    def on_limit_check_toggle(self):
        self.sim.should_check_limits = self.check_limits_box.checkState() == 2
