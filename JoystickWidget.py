from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPaintEvent, QPainter, QColor, QKeyEvent, QMouseEvent
from PyQt5.QtCore import Qt, QTimer, pyqtSignal


def clamp(value):
    return min(1, max(0, value))


class JoystickWidget(QWidget):
    DT = 1 / 60.0
    MOVE_SPEED = 1.0
    GAP = 5
    MAIN_SQUARE = 75

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

        timer = QTimer(self)
        timer.timeout.connect(self.every_dt)
        timer.start(self.DT * 1000)

    def every_dt(self):
        up_factor = (1 if self.up_pressed else 0) + (-1 if self.down_pressed else 0)
        right_factor = (1 if self.right_pressed else 0) + (-1 if self.left_pressed else 0)

        self.set_x_pos(self.x_pos + right_factor * self.MOVE_SPEED * self.DT)
        self.set_y_pos(self.y_pos + up_factor * self.MOVE_SPEED * self.DT)

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
        painter.setBrush(Qt.NoBrush)
        painter.save()
        painter.translate(self.GAP, self.GAP)
        painter.drawLine(virtual_x_orig * slider_long, 0, virtual_x_orig * slider_long, slider_short)
        painter.drawRect(0, 0, slider_long, slider_short)
        painter.setPen(Qt.red)
        painter.drawLine(self.x_pos * slider_long, 0, self.x_pos * slider_long, slider_short)
        painter.restore()

        painter.save()
        painter.translate(self.GAP + self.MAIN_SQUARE + self.GAP, self.GAP + slider_short + self.GAP)
        painter.drawRect(0, 0, slider_short, slider_long)
        painter.drawLine(0, virtual_y_orig * slider_long, slider_short, virtual_y_orig * slider_long)
        painter.setPen(Qt.red)
        painter.drawLine(0, self.y_pos * slider_long, slider_short, self.y_pos * slider_long)
        painter.restore()

    def keyChange(self, key, is_down):
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
        mouse_x = e.pos().x()
        mouse_y = e.pos().y()

        side = min(self.width(), self.height())
        x_orig = (self.width() - side) / 2
        y_orig = (self.height() - side) / 2

        x_m = 100.0 / (self.MAIN_SQUARE * side)
        self.set_x_pos(mouse_x * x_m - (x_orig + self.GAP * side / 100.0) * x_m)

        y_m = -100.0 / (self.MAIN_SQUARE * side)
        self.set_y_pos(mouse_y * y_m - (y_orig + (self.GAP + self.MAIN_SQUARE) * side / 100.0) * y_m)

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
