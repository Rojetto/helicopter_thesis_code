from typing import List

from PyQt5 import QtWidgets

from helicontrollers.AbstractController import *
from helicontrollers.util import FeedForwardMethod


class ControllerFrame(QtWidgets.QFrame):
    def __init__(self, controller_list: List[AbstractController]):
        super().__init__()

        self.controllers = controller_list

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setMinimumHeight(200)

        combos_layout = QtWidgets.QHBoxLayout()

        self.controller_combo = QtWidgets.QComboBox()
        self.controller_combo.currentIndexChanged.connect(self.on_controller_combo_select)
        self.feed_forward_combo = QtWidgets.QComboBox()
        self.feed_forward_combo.insertItem(0, "No feed-forward", FeedForwardMethod.NONE)
        self.feed_forward_combo.insertItem(1, "Static feed-forward", FeedForwardMethod.STATIC)
        self.feed_forward_combo.insertItem(2, "Flatness based feed-forward", FeedForwardMethod.FLATNESS)
        combos_layout.addWidget(self.controller_combo)
        combos_layout.addWidget(self.feed_forward_combo)
        self.controller_frame_stack = QtWidgets.QStackedWidget()

        # This is a list. Each element is a dictionary that holds parameter specifications for a specific controller.
        # These dictionaries use the parameter names as keys and store tuples (ParamSpec, QtWidget(s)) as values.
        # ParamBool       -->  QtWidgets.QCheckBox
        # ParamEnum       -->  QtWidgets.QComboBox
        # ParamFloatArray --> [QtWidgets.QDoubleSpinBox]
        self.controller_param_dicts_with_widgets = []

        for controller_i, controller in enumerate(controller_list):
            self.controller_combo.insertItem(controller_i, controller.name)
            this_controller_frame = QtWidgets.QFrame()
            this_controller_layout = QtWidgets.QFormLayout()
            this_controller_frame.setLayout(this_controller_layout)

            this_param_dict = {}

            param_i = 0
            for param_name, param_spec in controller.param_definition_dict.items():
                row_content = None
                widgets = None

                if type(param_spec) is ParamBool:
                    row_content = QtWidgets.QCheckBox()
                    row_content.setChecked(param_spec.init_value)
                    widgets = row_content
                elif type(param_spec) is ParamFloatArray:
                    widgets = [QtWidgets.QDoubleSpinBox() for _ in param_spec.init_values]
                    row_content = QtWidgets.QHBoxLayout()

                    for box_i, box in enumerate(widgets):
                        box.setRange(param_spec.mins[box_i], param_spec.maxs[box_i])
                        box.setValue(param_spec.init_values[box_i])
                        row_content.addWidget(box)

                elif type(param_spec) is ParamEnum:
                    row_content = QtWidgets.QComboBox()
                    widgets = row_content

                    for i in range(len(param_spec.option_labels)):
                        row_content.insertItem(i, param_spec.option_labels[i], param_spec.option_values[i])
                        row_content.setCurrentIndex(row_content.findData(param_spec.init_value))

                this_controller_layout.insertRow(param_i, param_name, row_content)

                this_param_dict[param_name] = (param_spec, widgets)
                param_i += 1

            self.controller_frame_stack.addWidget(this_controller_frame)
            self.controller_param_dicts_with_widgets.append(this_param_dict)

        scroll_area.setWidget(self.controller_frame_stack)

        main_layout.addWidget(scroll_area, 1)
        main_layout.addLayout(combos_layout)

    def on_controller_combo_select(self):
        self.controller_frame_stack.setCurrentIndex(self.controller_combo.currentIndex())

    def get_selected_controller_and_params(self):
        controller_i = self.controller_combo.currentIndex()
        selected_controller = self.controllers[controller_i]
        param_dict = self.controller_param_dicts_with_widgets[controller_i]

        param_values = {}

        for param_name, (param_spec, widgets) in param_dict.items():
            value = None

            if type(param_spec) is ParamBool:
                value = widgets.checkState() == 2  # Qt.CheckState is either 0 (unchecked) or 2 (checked)
            elif type(param_spec) is ParamFloatArray:
                value = [widget.value() for widget in widgets]
            elif type(param_spec) is ParamEnum:
                value = widgets.currentData()

            param_values[param_name] = value

        return selected_controller, param_values

    def get_selected_feed_forward_method(self):
        return self.feed_forward_combo.currentData()
