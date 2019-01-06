from enum import Enum
from inputs import devices
from threading import Thread

from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPaintEvent, QPainter, QColor, QKeyEvent, QMouseEvent, QFont
from PyQt5.QtCore import Qt, QTimer, pyqtSignal


class InputDevice(Enum):
    KEYBOARD = 0
    MOUSE = 1
    GAMEPAD_LEFT_STICK = 2
    GAMEPAD_LEFT_STICK_AND_TRIGGER = 3


def clamp(value):
    return min(1, max(0, value))


class JoystickWidget(QWidget):
    DT = 1 / 60.0
    MOVE_SPEED = 1.0
    GAP = 8
    MAIN_SQUARE = 70

    GAMEPAD_DEADZONE = 0.05

    pos_changed = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.ClickFocus)  # VERY IMPORTANT, otherwise we don't receive keyboard events

        self.up_pressed = False
        self.down_pressed = False
        self.left_pressed = False
        self.right_pressed = False

        self.real_x_min = -100
        self.real_x_max = 100
        self.real_y_min = -100
        self.real_y_max = 100

        self.x_pos = 0.0  # 0..1, 0 is left
        self.y_pos = 0.0  # 0..1, 0 is bottom

        self.reset_pos()

        self.input_device = InputDevice.MOUSE

        self.gamepad = Gamepad()
        if self.gamepad.connected:
            print("Gamepad connected")
        else:
            print("No gamepad connected")

        timer = QTimer(self)
        timer.timeout.connect(self.every_dt)
        timer.start(self.DT * 1000)

    def every_dt(self):
        if self.gamepad.connected:
            if self.gamepad.trig_right > 0:
                self.input_device = InputDevice.GAMEPAD_LEFT_STICK_AND_TRIGGER
            elif abs(self.gamepad.stick_left_y) > 0.3:
                self.input_device = InputDevice.GAMEPAD_LEFT_STICK

        if self.input_device == InputDevice.KEYBOARD:
            up_factor = (1 if self.up_pressed else 0) + (-1 if self.down_pressed else 0)
            right_factor = (1 if self.right_pressed else 0) + (-1 if self.left_pressed else 0)

            self.set_x_pos(self.x_pos + right_factor * self.MOVE_SPEED * self.DT)
            self.set_y_pos(self.y_pos + up_factor * self.MOVE_SPEED * self.DT)
        elif (self.input_device == InputDevice.GAMEPAD_LEFT_STICK
              or self.input_device == InputDevice.GAMEPAD_LEFT_STICK_AND_TRIGGER):
            in_x = self.gamepad.stick_left_x
            if self.input_device == InputDevice.GAMEPAD_LEFT_STICK:
                in_y = self.gamepad.stick_left_y
            else:
                in_y = self.gamepad.trig_right

            in_x = in_x if abs(in_x) > self.GAMEPAD_DEADZONE else 0
            in_y = in_y if abs(in_y) > self.GAMEPAD_DEADZONE else 0

            virtual_x_orig, virtual_y_orig = self.real_to_virtual(0, 0)

            if in_x > 0:
                virtual_x = virtual_x_orig + (1 - virtual_x_orig) * in_x
            else:
                virtual_x = virtual_x_orig + virtual_x_orig * in_x

            if in_y > 0:
                virtual_y = virtual_y_orig + (1 - virtual_y_orig) * in_y
            else:
                virtual_y = virtual_y_orig + virtual_y_orig * in_y

            self.set_x_pos(virtual_x)
            self.set_y_pos(virtual_y)

        self.update()

    def paintEvent(self, e: QPaintEvent):
        painter = QPainter(self)

        light_green = QColor(204, 255, 204)
        dark_green = QColor(0, 128, 0)

        # set up a square coordinate system from 0 to 100 in each axis, origin in bottom left, y pointing up
        side = min(self.width(), self.height())
        painter.translate((self.width() - side) / 2, (self.height() + side) / 2)
        painter.scale(side / 100, -side / 100)
        painter.setRenderHint(QPainter.Antialiasing)

        slider_long = self.MAIN_SQUARE
        slider_short = 100 - self.MAIN_SQUARE - 3 * self.GAP
        dot_radius = 2

        painter.setPen(dark_green)
        painter.setBrush(light_green)

        virtual_x_orig, virtual_y_orig = self.real_to_virtual(0, 0)

        # main square
        painter.save()
        # translate the coordinate system to the bottom left of the main square
        painter.translate(self.GAP, self.GAP + slider_short + self.GAP)
        painter.drawRect(0, 0, self.MAIN_SQUARE, self.MAIN_SQUARE)
        # current position
        painter.setBrush(Qt.red)
        painter.setPen(Qt.red)
        painter.drawEllipse(self.x_pos * self.MAIN_SQUARE - dot_radius, self.y_pos * self.MAIN_SQUARE - dot_radius, 2*dot_radius, 2*dot_radius)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(dark_green)
        painter.drawRect(0, 0, self.MAIN_SQUARE, self.MAIN_SQUARE)
        # vertical origin line
        painter.drawLine(virtual_x_orig * self.MAIN_SQUARE, 0, virtual_x_orig * self.MAIN_SQUARE, self.MAIN_SQUARE)
        # horizontal origin line
        painter.drawLine(0, virtual_y_orig * self.MAIN_SQUARE, self.MAIN_SQUARE, virtual_y_orig * self.MAIN_SQUARE)
        painter.restore()

        # sliders
        real_x, real_y = self.virtual_to_real(self.x_pos, self.y_pos)
        real_x_string = f"{real_x:.2f}"  # print with 2 decimal places
        real_y_string = f"{real_y:.2f}"
        font = QFont("Courier New", 3)
        font.setBold(True)
        painter.setFont(font)

        painter.setBrush(Qt.NoBrush)
        painter.save()
        painter.translate(self.GAP, self.GAP)
        painter.drawLine(virtual_x_orig * slider_long, 0, virtual_x_orig * slider_long, slider_short)
        painter.drawRect(0, 0, slider_long, slider_short)
        painter.setPen(Qt.red)
        painter.drawLine(self.x_pos * slider_long, 0, self.x_pos * slider_long, slider_short)
        painter.scale(1, -1)  # text drawing expects y to point "down"
        painter.drawText(0, -2*slider_short - 1, slider_long, slider_short, Qt.AlignCenter, real_x_string)
        painter.restore()

        painter.save()
        painter.translate(self.GAP + self.MAIN_SQUARE + self.GAP, self.GAP + slider_short + self.GAP)
        painter.drawRect(0, 0, slider_short, slider_long)
        painter.drawLine(0, virtual_y_orig * slider_long, slider_short, virtual_y_orig * slider_long)
        painter.setPen(Qt.red)
        painter.drawLine(0, self.y_pos * slider_long, slider_short, self.y_pos * slider_long)
        painter.scale(1, -1)
        painter.rotate(-90)
        painter.drawText(0, -slider_short - 1, slider_long, slider_short, Qt.AlignCenter, real_y_string)
        painter.restore()

    def keyChange(self, key, is_down):
        self.input_device = InputDevice.KEYBOARD

        if key == Qt.Key_Up or key == Qt.Key_W:
            self.up_pressed = is_down
        elif key == Qt.Key_Down or key == Qt.Key_S:
            self.down_pressed = is_down
        elif key == Qt.Key_Left or key == Qt.Key_A:
            self.left_pressed = is_down
        elif key == Qt.Key_Right or key == Qt.Key_D:
            self.right_pressed = is_down

    def keyPressEvent(self, e: QKeyEvent):
        self.keyChange(e.key(), True)

    def keyReleaseEvent(self, e: QKeyEvent):
        self.keyChange(e.key(), False)

    def real_to_virtual(self, real_x, real_y):
        virtual_x = (real_x - self.real_x_min) / (self.real_x_max - self.real_x_min)
        virtual_y = (real_y - self.real_y_min) / (self.real_y_max - self.real_y_min)

        return virtual_x, virtual_y

    def virtual_to_real(self, virtual_x, virtual_y):
        real_x = self.real_x_min + virtual_x * (self.real_x_max - self.real_x_min)
        real_y = self.real_y_min + virtual_y * (self.real_y_max - self.real_y_min)

        return real_x, real_y

    def reset_pos(self):
        x, y = self.real_to_virtual(0, 0)
        self.set_x_pos(x)
        self.set_y_pos(y)

    def mouseMoveEvent(self, e: QMouseEvent):
        self.input_device = InputDevice.MOUSE

        mouse_x = e.pos().x()
        mouse_y = e.pos().y()

        side = min(self.width(), self.height())
        x_orig = (self.width() - side) / 2
        y_orig = (self.height() - side) / 2

        x_m = 100.0 / (self.MAIN_SQUARE * side)
        self.set_x_pos(mouse_x * x_m - (x_orig + self.GAP * side / 100.0) * x_m)

        y_m = -100.0 / (self.MAIN_SQUARE * side)
        self.set_y_pos(mouse_y * y_m - (y_orig + (self.GAP + self.MAIN_SQUARE) * side / 100.0) * y_m)

    def mouseReleaseEvent(self, e: QMouseEvent):
        self.input_device = InputDevice.MOUSE
        self.reset_pos()

    def set_x_pos(self, new_x_pos):
        clamped = clamp(new_x_pos)

        if clamped != self.x_pos:
            self.x_pos = clamped
            real_x, real_y = self.virtual_to_real(self.x_pos, self.y_pos)
            self.pos_changed.emit(real_x, real_y)

    def set_y_pos(self, new_y_pos):
        clamped = clamp(new_y_pos)

        if clamped != self.y_pos:
            self.y_pos = clamped
            real_x, real_y = self.virtual_to_real(self.x_pos, self.y_pos)
            self.pos_changed.emit(real_x, real_y)


class Gamepad:
    def __init__(self):
        self.connected = False
        self.device = None

        self.btn_a_down = False
        self.btn_a_pressed = False
        self.btn_b_down = False
        self.btn_b_pressed = False
        self.btn_x_down = False
        self.btn_x_pressed = False
        self.btn_y_down = False
        self.btn_y_pressed = False

        self.trig_right = 0
        self.trig_left = 0

        self.stick_left_x = 0
        self.stick_left_y = 0
        self.stick_right_x = 0
        self.stick_right_y = 0

        connected_gamepads = devices.gamepads
        if len(connected_gamepads) > 0:
            self.device = connected_gamepads[0]
            self.connected = True

        self.thread = Thread(target=self._read_event_loop)
        self.thread.start()

    def _read_event_loop(self):
        if not self.connected:
            return

        while True:
            # This "read" call blocks when there are no queued events
            events = self.device.read()
            for e in events:
                code = e.code
                val = e.state

                if code == "BTN_SOUTH":
                    if val and not self.btn_a_down:
                        self.btn_a_pressed = True
                    self.btn_a_down = val == 1
                if code == "BTN_EAST":
                    if val and not self.btn_b_down:
                        self.btn_b_pressed = True
                    self.btn_b_down = val == 1
                if code == "BTN_WEST":
                    if val and not self.btn_x_down:
                        self.btn_x_pressed = True
                    self.btn_x_down = val == 1
                if code == "BTN_NORTH":
                    if val and not self.btn_y_down:
                        self.btn_y_pressed = True
                    self.btn_y_down = val == 1

                if code == "ABS_Z":
                    self.trig_left = val / 255.0
                if code == "ABS_RZ":
                    self.trig_right = val / 255.0

                if code == "ABS_X":
                    self.stick_left_x = val / 32768.0
                if code == "ABS_Y":
                    self.stick_left_y = val / 32768.0
                if code == "ABS_RX":
                    self.stick_right_x = val / 32768.0
                if code == "ABS_RY":
                    self.stick_right_y = val / 32768.0

    def reset_pressed(self):
        """
        Reset all "pressed" variables to False so that you get accurate values in the next frame
        """
        self.btn_a_pressed = False
        self.btn_b_pressed = False
        self.btn_x_pressed = False
        self.btn_y_pressed = False
