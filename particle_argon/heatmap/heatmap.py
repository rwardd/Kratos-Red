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

HEIGHT = 240
WIDTH = 240
COLOUR_DEPTH = 128
MIN_TEMP = 25.0
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


data = np.zeros((8, 8))
s = serial.Serial("/dev/ttyACM0", 115200)
app = QtWidgets.QApplication(sys.argv)


def print_grid():
    print("ROW8: ", row8)
    print("ROW7: ", row7)
    print("ROW6: ", row6)
    print("ROW5: ", row5)
    print("ROW4: ", row4)
    print("ROW3: ", row3)
    print("ROW2: ", row2)
    print("ROW1: ", row1)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")


def heatmap():

    data[0] = row8
    data[1] = row7
    data[2] = row6
    data[3] = row5
    data[4] = row4
    data[5] = row3
    data[6] = row2
    data[7] = row1
    print(data)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    #rearrange data for heatmap
    data[0] = row1
    data[1] = row2
    data[2] = row3
    data[3] = row4
    data[4] = row5
    data[5] = row6
    data[6] = row7
    data[7] = row8

    #BICUBIC --- FOR HEATMAP
    points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0,64)]
    gridX, gridY = np.mgrid[0:7:32j, 0:7:32j]
    bicubic = griddata(points, np.reshape(data, (64,)), (gridX, gridY), method="cubic")   
    #END BICUBIC

    
    
    window.show()
    image.setImage(data)


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
            row1[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)
        elif index == '1':
            row2[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)
        elif index == '2':
            row3[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)
        elif index == '3':
            row4[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)
        elif index == '4':
            row5[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)
        elif index == '5':
            row6[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)
        elif index == '6':
            row7[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)
        elif index == '7':
            row8[i] = map_value(float(line[i]), MIN_TEMP,
                                MAX_TEMP, 0, COLOUR_DEPTH-1)

    heatmap()
    # print_grid()


timer = QtCore.QTimer()
timer.setInterval(5)
timer.timeout.connect(read_loop)
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
