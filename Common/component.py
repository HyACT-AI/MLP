# python   3.7.0
# PyQt5    5.15.1

# 제목 : Componet Package v1.0.1
# 날짜 : 2021.01.04
# 내용 : Car Class 수정

import os
import math

from PyQt5 import QtWidgets, QtGui, QtCore

class Title(QtWidgets.QFrame):

    def __init__(self, title='', first_line='', second_line=''):
        super().__init__()  

        title_label = QtWidgets.QLabel(title, self)
        title_label.setAlignment(QtCore.Qt.AlignCenter)
        title_label.setStyleSheet(
            'color: white;'
            'font: bold 50px Tahoma;'
            )
        main_label = QtWidgets.QLabel(first_line, self)
        main_label.setAlignment(QtCore.Qt.AlignCenter)
        main_label.setStyleSheet(
            'color: white;'
            'font: 25px Tahoma;'
            )
        sub_label = QtWidgets.QLabel(second_line, self)
        sub_label.setAlignment(QtCore.Qt.AlignCenter)
        sub_label.setStyleSheet(
            'color: white;'
            'font: 25px Tahoma;'
            )

        self.prev_btn = QtWidgets.QPushButton('', self)
        self.prev_btn.setFixedSize(50, 50)
        self.prev_btn.setStyleSheet(
            'color: white;'
            'background-color: rgb(23, 85, 218);'
            'image:url(./Common/Images/enable_left_arrow.png);'
            'border-radius: 5px;'
            )
        self.next_btn = QtWidgets.QPushButton('', self)
        self.next_btn.setFixedSize(50, 50)
        self.next_btn.setStyleSheet(
            'color: white;'
            'background-color: rgb(23, 85, 218);'
            'image:url(./Common/Images/enable_right_arrow.png);'
            'border-radius: 5px;'
            )
        btn_hbox = QtWidgets.QHBoxLayout()
        btn_hbox.addStretch(3)
        btn_hbox.addWidget(self.prev_btn)
        btn_hbox.addWidget(self.next_btn)
        
        self.vbox = QtWidgets.QVBoxLayout()
        self.vbox.addWidget(title_label)
        self.vbox.addLayout(btn_hbox)
        self.vbox.addWidget(main_label)
        self.vbox.addWidget(sub_label)

    def setEnablePreviousButton(self, enabled):
        if enabled == False:            
            self.prev_btn.setStyleSheet(            
                'color: rgb(200, 200, 200);'
                'background-color: rgb(25, 110, 210);'
                'image:url(./Common/Images/disable_left_arrow.png);'
                'border-radius: 5px;'
                )
        else:
            self.prev_btn.setStyleSheet(
            'color: white;'
            'background-color: rgb(23, 85, 218);'
            'image:url(./Common/Images/enable_left_arrow.png);'
            'border-radius: 5px;'
            )
    
    def setEnableNextButton(self, enabled):
        if enabled == False:
            self.next_btn.setStyleSheet(                
                'color: white;'
                'background-color: rgb(23, 85, 218);'
                'image:url(./Common/Images/disable_right_arrow.png);'
                'border-radius: 5px;'
                )
        else:            
            self.next_btn.setStyleSheet(
                'color: white;'
                'background-color: rgb(23, 85, 218);'
                'image:url(./Common/Images/inable_right_arrow.png);'
                'border-radius: 5px;'
                )

class HLine(QtWidgets.QFrame):
    def __init__(self, color=QtGui.QColor(58, 134, 255)):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.HLine)
        self.setFrameShadow(QtWidgets.QFrame.Plain)
        self.setLineWidth(0)
        self.setMidLineWidth(3)
        self.setContentsMargins(0, 0, 0, 0)

        pal = self.palette()
        pal.setColor(QtGui.QPalette.WindowText, color)
        self.setPalette(pal)   

class RangeSlider(QtWidgets.QWidget):
    def __init__(self):
        super().__init__(None)

        self.first_position = 0
        self.second_position = 50

        self.opt = QtWidgets.QStyleOptionSlider()
        self.opt.minimum = 0
        self.opt.maximum = 255

        self.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.setTickInterval(1)

        self.setSizePolicy(
            QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Slider)
        )

        self.sizeWeight = self.width() / self.opt.maximum
        self.first_position_label = QtWidgets.QLabel(str(self.first_position), self)
        self.first_position_label.resize(30, 30)
        self.second_position_label = QtWidgets.QLabel(str(self.second_position), self)
        self.second_position_label.resize(30, 30)
        self.updatePositionLabel()

    def setRangeLimit(self, minimum: int, maximum: int):
        self.opt.minimum = minimum
        self.opt.maximum = maximum
        self.sizeWeight = self.width() / self.opt.maximum
        self.update()

    def setRange(self, start: int, end: int):
        self.first_position = start
        self.second_position = end
        self.update()

    def getRange(self):
        return (self.first_position, self.second_position)

    def setTickPosition(self, position: QtWidgets.QSlider.TickPosition):
        self.opt.tickPosition = position

    def setTickInterval(self, ti: int):
        self.opt.tickInterval = ti
    
    def resizeEvent(self, event: QtGui.QResizeEvent):
        self.sizeWeight = self.width() / self.opt.maximum
        self.updatePositionLabel()

    def paintEvent(self, event: QtGui.QPaintEvent):
        painter = QtGui.QPainter(self)
        # Draw rule
        self.opt.initFrom(self)
        self.opt.rect = self.rect()
        self.opt.sliderPosition = 0
        self.opt.subControls = QtWidgets.QStyle.SC_SliderGroove | QtWidgets.QStyle.SC_SliderTickmarks
        #   Draw GROOVE
        self.style().drawComplexControl(QtWidgets.QStyle.CC_Slider, self.opt, painter)
        #  Draw INTERVAL
        color = self.palette().color(QtGui.QPalette.Highlight)
        color.setAlpha(160)
        painter.setBrush(QtGui.QBrush(color))
        painter.setPen(QtCore.Qt.NoPen)

        self.opt.sliderPosition = self.first_position
        x_left_handle = (
            self.style()
            .subControlRect(QtWidgets.QStyle.CC_Slider, self.opt, QtWidgets.QStyle.SC_SliderHandle)
            .right()
        )
        self.opt.sliderPosition = self.second_position
        x_right_handle = (
            self.style()
            .subControlRect(QtWidgets.QStyle.CC_Slider, self.opt, QtWidgets.QStyle.SC_SliderHandle)
            .left()
        )
        groove_rect = self.style().subControlRect(
            QtWidgets.QStyle.CC_Slider, self.opt, QtWidgets.QStyle.SC_SliderGroove
        )
        selection = QtCore.QRect(
            x_left_handle,
            groove_rect.y(),
            x_right_handle - x_left_handle,
            groove_rect.height(),
        ).adjusted(-1, 1, 1, -1)
        painter.drawRect(selection)
        # Draw first handle
        self.opt.subControls = QtWidgets.QStyle.SC_SliderHandle
        self.opt.sliderPosition = self.first_position
        self.style().drawComplexControl(QtWidgets.QStyle.CC_Slider, self.opt, painter)
        # Draw second handle
        self.opt.sliderPosition = self.second_position
        self.style().drawComplexControl(QtWidgets.QStyle.CC_Slider, self.opt, painter)

        self.updatePositionLabel()

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        self.opt.sliderPosition = self.first_position
        self._first_sc = self.style().hitTestComplexControl(
            QtWidgets.QStyle.CC_Slider, self.opt, event.pos(), self
        )
        self.opt.sliderPosition = self.second_position
        self._second_sc = self.style().hitTestComplexControl(
            QtWidgets.QStyle.CC_Slider, self.opt, event.pos(), self
        )

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        distance = self.opt.maximum - self.opt.minimum
        pos = self.style().sliderValueFromPosition(
            0, distance, event.pos().x(), self.rect().width()
        )
        if self._first_sc == QtWidgets.QStyle.SC_SliderHandle:
            if pos <= self.second_position:
                self.first_position = pos
                self.update()
                return
        if self._second_sc == QtWidgets.QStyle.SC_SliderHandle:
            if pos >= self.first_position:
                self.second_position = pos
                self.update()

    def sizeHint(self):
        """ override """
        SliderLength = 84
        TickSpace = 5

        w = SliderLength
        h = self.style().pixelMetric(QtWidgets.QStyle.PM_SliderThickness, self.opt, self)

        if ((self.opt.tickPosition & QtWidgets.QSlider.TicksAbove) or
            (self.opt.tickPosition & QtWidgets.QSlider.TicksBelow)):
            h += TickSpace

        return (
            self.style()
            .sizeFromContents(QtWidgets.QStyle.CT_Slider, self.opt, QtCore.QSize(w, h), self)
            .expandedTo(QtWidgets.QApplication.globalStrut())
        )
    
    def _setLabelX(self, index, value):
        pos = value * self.sizeWeight
        add = self.sizeWeight * value * 0.015
        if (10 <= value) and (value < 100):
            add += 2.5 * self.sizeWeight
        elif value >= 100:
            add += 5 * self.sizeWeight
        return int(pos - add)
    
    def updatePositionLabel(self):
        self.first_position_label.setText(str(self.first_position))
        self.first_position_label.move(self._setLabelX(0, self.first_position), 0)  
        self.second_position_label.setText(str(self.second_position))
        self.second_position_label.move(self._setLabelX(1, self.second_position), 0)

class Car(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        
        self.img = QtGui.QImage('./Common/Images/car.png')
        self.position = [0, 0]
        self.center = [-24.5, -7.5]
        self.angle = 0
        self.s_weight = [
            math.atan2(-38.8333, 30),
            math.atan2(-16.7742, 30),
            math.atan2(0, 30),
            math.atan2(16.7742, 30),
            math.atan2(38.8333, 30)
            ]
        self.b_weight = math.pi / 2
        self.r = 0.0
        self.bump = False
        self.bumper = []
        for i in range(15):
            self.bumper.append([0, 0])
        self.sensor = []
        self.detection_distance = []
        for i in range(5):
            self.detection_distance.append(0)
            self.sensor.append([0, 0])
    
    def move(self):
        self.r = math.radians(self.angle)
        x = math.cos(self.r)
        y = math.sin(self.r)
        self.position[0] += x
        self.position[1] += y

    def updateSensor(self, background):
        for i in range(5):
            r = self.r + self.s_weight[i]
            for d in range(1, 101):
                self.sensor[i][0] = int(d * math.cos(r) + self.position[0])
                self.sensor[i][1] = int(d * math.sin(r) + self.position[1])
                if (background.pixel(self.sensor[i][0], self.sensor[i][1]) == QtGui.qRgb(150, 150, 150)) or (d == 100):
                    self.detection_distance[i] = d
                    break
    
    def updateBumper(self, background):
        for i in range(15):
            if i < 8:
                r = self.r - self.b_weight
                self.bumper[i][0] = int((7.5 - i) * math.cos(r) + self.position[0])
                self.bumper[i][1] = int((7.5 - i) * math.sin(r) + self.position[1])
            else:
                r = math.radians(self.angle) + self.b_weight
                self.bumper[i][0] = int((i - 7.5) * math.cos(r) + self.position[0])
                self.bumper[i][1] = int((i - 7.5) * math.sin(r) + self.position[1])
            if background.pixel(self.bumper[i][0], self.bumper[i][1]) == QtGui.qRgb(150, 150, 150):
                self.bump = True
                break