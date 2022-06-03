## SIGNAL QUALITY INDICATOR ##
import sys
import os
import serial
import time
import threading
import readchar
import json
import tkinter as tk
import numpy as np
import sys
import matplotlib as mpl
import matplotlib.pyplot as plt
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from colour import Color
from pgcolorbar.colorlegend import ColorLegendItem
import cv2
from scipy.interpolate import griddata
import math


# MACROS #
FREQUENCY = 10  # Hz
WINDOW_LENGTH = 30  # seconds
M = FREQUENCY * WINDOW_LENGTH  # points
HEIGHT = 240
WIDTH = 240
COLOUR_DEPTH = 1024
MIN_TEMP = 26.0
MAX_TEMP = 32.0
DISPLAY_PIXEL_HEIGHT = HEIGHT/30
DISPLAY_PIXEL_WIDTH = WIDTH/30


def map_value(x, inMin, inMax, outMin, outMax):
    if x < MIN_TEMP:
        return 0
    elif x > MAX_TEMP:
        return COLOUR_DEPTH - 1
    else:
        return int((x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin)


row1 = [0, 0, 0, 0, 0, 0, 0, 0]
row2 = [0, 0, 0, 0, 0, 0, 0, 0]
row3 = [0, 0, 0, 0, 0, 0, 0, 0]
row4 = [0, 0, 0, 0, 0, 0, 0, 0]
row5 = [0, 0, 0, 0, 0, 0, 0, 0]
row6 = [0, 0, 0, 0, 0, 0, 0, 0]
row7 = [0, 0, 0, 0, 0, 0, 0, 0]
row8 = [0, 0, 0, 0, 0, 0, 0, 0]


p1 = np.zeros((64, M))
sqi = np.zeros((8, 8))
count = 0


hann = np.hanning(M)
data = np.zeros((8, 8))
s = serial.Serial("/dev/ttyACM0", 115200)
app = QtWidgets.QApplication(sys.argv)


def calibrate():
    # count to 100 and insert values into p1
    data1D = np.reshape(data, 64)
    for i in range(64):
        p1[i][count] = data1D[i]


def append_window():
    data1D = np.reshape(data, 64)
    global p1
    p1 = np.roll(p1, -1, axis=1)
    for i in range(64):
        p1[i][M-1] = data1D[i]


def sqi_calc():
    # array = array of temp samples of 100 samples
    global sqi
    sqi = np.reshape(sqi, 64)
    for i in range(64):
        windowed = np.multiply(p1[i], hann)
        for j in range(16):
            windowed = np.delete(windowed, 0)
        for k in range(33):
            windowed = np.delete(windowed, -1)

        fft = np.fft.fft(windowed)

        arrayABS = np.absolute(fft)

        arrayP = np.square(arrayABS)

        maximum = np.amax(arrayABS)

        denominator = math.sqrt(np.sum(arrayP))

        sqi[i] = maximum/denominator
    sqi = np.reshape(sqi, (8, 8))


def print_sqi():
    print(sqi)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    window.show()
    image.setImage(sqi)


def fill_data():
    data[0] = row1
    data[1] = row2
    data[2] = row3
    data[3] = row4
    data[4] = row5
    data[5] = row6
    data[6] = row7
    data[7] = row8


def read_loop():

    data = s.readline()
    # print(data)
    data = data.decode()
    # print(str(data))
    line = data.split(',')
    # print(line)
    line = line[:-1]
    index = line[0]
    del line[0]

    for i in range(8):
        if index == '0':
            row1[i] = float(line[i])
        elif index == '1':
            row2[i] = float(line[i])
        elif index == '2':
            row3[i] = float(line[i])
        elif index == '3':
            row4[i] = float(line[i])
        elif index == '4':
            row5[i] = float(line[i])
        elif index == '5':
            row6[i] = float(line[i])
        elif index == '6':
            row7[i] = float(line[i])
        elif index == '7':
            row8[i] = float(line[i])


# samples - sample temps? for each pixel its gonna be M points
#samples = np.random.default_rng().uniform(26, 32, M)
#fakeSamples = np.random.default_rng().uniform(21, 24, M)
#samples = samples*hann

#samples = np.fft.fft(samples)
#samplesABS = np.absolute(samples)
#samplesP = np.square(samplesABS)

#fakeSamples = np.fft.fft(fakeSamples)
#fakeSamplesABS = np.absolute(fakeSamples)
#fakeSamplesP = np.square(fakeSamplesABS)


# print(sqi(fakeSamples))

# pylab.plot(fakeSamplesP)
# pylab.show()

def heatmap():
    global count
    read_loop()
    fill_data()
    if count < M:
        calibrate()
        count += 1
    else:
        # maybe calc SQI first then append? no append then calc cuz we filled data
        append_window()
        # now calculate sqi
        sqi_calc()
        print_sqi()


timer = QtCore.QTimer()
timer.setInterval(5)
timer.timeout.connect(heatmap)
timer.start()


window = pg.GraphicsLayoutWidget()
blue, red = Color('blue'), Color('Red')
colors = blue.range_to(red, COLOUR_DEPTH)
colors_array = np.array([np.array(color.get_rgb())*255 for color in colors])
#colors_array = np.append(colors_array, [[255, 255, 255]], axis=0)
look_up_table = colors_array.astype(np.uint8)


image = pg.ImageItem()
image.setOpts(axisOrder='row-major')
image.setLookupTable(look_up_table)
image.setImage(data)

view_box = pg.ViewBox()
view_box.setAspectLocked(lock=True)
view_box.addItem(image)

plot = pg.PlotItem(viewBox=view_box)
window.addItem(plot)
sys.exit(app.exec_())
