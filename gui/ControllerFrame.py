from typing import List
from gui.ParameterFrame import ParameterFrame
from helicontrollers.AbstractController import AbstractController


class ControllerFrame(ParameterFrame):
    def __init__(self, controller_list: List[AbstractController]):
        super().__init__(controller_list)
