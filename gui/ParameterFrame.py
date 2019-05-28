from typing import List
from PyQt5 import QtWidgets
from abc import ABC


class ParamBool:
    def __init__(self, init_value):
        self.init_value = init_value


class ParamFloatArray:
    def __init__(self, init_values):
        self.init_values = init_values


class ParamEnum:
    def __init__(self, option_labels, option_values, init_value):
        self.option_labels = option_labels
        self.option_values = option_values
        self.init_value = init_value


class ParameterizedClass(ABC):
    def __init__(self, name, param_definition_dict):
        self.name = name
        self.param_definition_dict = param_definition_dict

    def set_params(self, param_value_dict):
        raise NotImplementedError


class ParameterFrame(QtWidgets.QFrame):
    def __init__(self, object_list: List[ParameterizedClass]):
        super().__init__()

        self.objects = object_list

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setMinimumHeight(200)
        scroll_area.setMinimumWidth(950)

        combos_layout = QtWidgets.QHBoxLayout()

        self.object_combo = QtWidgets.QComboBox()
        self.object_combo.currentIndexChanged.connect(self.on_object_combo_select)
        combos_layout.addWidget(self.object_combo)
        self.object_frame_stack = QtWidgets.QStackedWidget()

        # This is a list. Each element is a dictionary that holds parameter specifications for a specific object.
        # These dictionaries use the parameter names as keys and store tuples (ParamSpec, QtWidget(s)) as values.
        # ParamBool       -->  QtWidgets.QCheckBox
        # ParamEnum       -->  QtWidgets.QComboBox
        # ParamFloatArray --> [QtWidgets.QDoubleSpinBox]
        self.object_param_dicts_with_widgets = []

        for object_i, object in enumerate(object_list):
            self.object_combo.insertItem(object_i, object.name)
            this_object_frame = QtWidgets.QFrame()
            this_object_layout = QtWidgets.QFormLayout()
            this_object_frame.setLayout(this_object_layout)

            this_param_dict = {}

            param_i = 0
            for param_name, param_spec in object.param_definition_dict.items():
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
                        box.setRange(-9999, 9999)
                        box.setValue(param_spec.init_values[box_i])
                        row_content.addWidget(box)

                elif type(param_spec) is ParamEnum:
                    row_content = QtWidgets.QComboBox()
                    widgets = row_content

                    for i in range(len(param_spec.option_labels)):
                        row_content.insertItem(i, param_spec.option_labels[i], param_spec.option_values[i])
                        row_content.setCurrentIndex(row_content.findData(param_spec.init_value))

                this_object_layout.insertRow(param_i, param_name, row_content)

                this_param_dict[param_name] = (param_spec, widgets)
                param_i += 1

            self.object_frame_stack.addWidget(this_object_frame)
            self.object_param_dicts_with_widgets.append(this_param_dict)

        scroll_area.setWidget(self.object_frame_stack)

        main_layout.addWidget(scroll_area, 1)
        main_layout.addLayout(combos_layout)

    def select_object(self, obj: ParameterizedClass):
        index = self.object_combo.findText(obj.name)
        self.object_combo.setCurrentIndex(index)

    def on_object_combo_select(self):
        self.object_frame_stack.setCurrentIndex(self.object_combo.currentIndex())

    def get_selected_object(self):
        object_i = self.object_combo.currentIndex()
        selected_object = self.objects[object_i]
        param_dict = self.object_param_dicts_with_widgets[object_i]

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

        selected_object.set_params(param_values)
        return selected_object
