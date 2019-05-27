from PyQt5 import QtWidgets, QtGui
import numpy as np
from ModelConstants import ModelType
from helicontrollers.util import FeedForwardMethod


class FeedforwardFrame(QtWidgets.QFrame):
    def __init__(self):
        # QtWidgets.QFrame.__init__(self)
        super().__init__()

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setFixedHeight(200)

        self.feedforward_combo = QtWidgets.QComboBox()
        self.feedforward_frame_stack = QtWidgets.QStackedWidget()

        # Flatness-based Feedforward
        self.feedforward_combo.insertItem(1, "Flatness-based Feedforward")
        flatness_frame = QtWidgets.QFrame()
        flatness_layout = QtWidgets.QFormLayout()
        flatness_frame.setLayout(flatness_layout)

        self.flatness_model_type = QtWidgets.QComboBox()
        self.flatness_model_type.addItems(["simple", "centripetal"])

        flatness_layout.addRow(QtWidgets.QLabel("Flatness-based feedforward model type: "),
                                self.flatness_model_type)

        self.feedforward_frame_stack.addWidget(flatness_frame)

        # Static Feedforward
        self.feedforward_combo.insertItem(2, "Static Feedforward")
        static_frame = QtWidgets.QFrame()
        static_layout = QtWidgets.QFormLayout()
        static_frame.setLayout(static_layout)

        static_layout.addRow(QtWidgets.QLabel("Static feedforward: "), QtWidgets.QLabel("---"))

        self.feedforward_frame_stack.addWidget(static_frame)

        #No Feedforward
        self.feedforward_combo.insertItem(3, "No Feedforward")
        nofeedforward_frame = QtWidgets.QFrame()
        nofeedforward_layout = QtWidgets.QFormLayout()
        nofeedforward_frame.setLayout(nofeedforward_layout)

        nofeedforward_layout.addRow(QtWidgets.QLabel("No Feedforward: "), QtWidgets.QLabel("---"))

        self.feedforward_frame_stack.addWidget(nofeedforward_frame)

        # Finishing touches
        scroll_area.setWidget(self.feedforward_frame_stack)
        main_layout.addWidget(scroll_area, 1)
        main_layout.addWidget(self.feedforward_combo)
        self.feedforward_combo.currentIndexChanged.connect(self.on_feedforward_combo_select)
        self.feedforward_combo.setCurrentIndex(2)


    def on_feedforward_combo_select(self):
        self.feedforward_frame_stack.setCurrentIndex(self.feedforward_combo.currentIndex())

    def get_feedforward_method_and_model(self):
        """:return model type"""
        combo_idx = self.feedforward_combo.currentIndex()
        if combo_idx == 0:
            print("Feedforward ==> Flatness")
            model_name = self.flatness_model_type.currentText()
            if model_name == "simple":
                model_type = ModelType.EASY
            elif model_name == "centripetal":
                model_type = ModelType.CENTRIPETAL
            return FeedForwardMethod.FLATNESS, model_type
        elif combo_idx == 1:
            print("Feedforward ==> Static")
            return FeedForwardMethod.STATIC, None
        elif combo_idx == 2:
            print("No Feedforward")
            return FeedForwardMethod.NONE, None
        else:
            raise Exception("FeedwordFrame.get_disturbance(): ComboxBox element ID was not recognized.")