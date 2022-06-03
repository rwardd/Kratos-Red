## RR detection using haar like filter ##
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
import pylab
from scipy.signal import butter, lfilter, decimate, argrelextrema


row1 = [0, 0, 0, 0, 0, 0, 0, 0]
row2 = [0, 0, 0, 0, 0, 0, 0, 0]
row3 = [0, 0, 0, 0, 0, 0, 0, 0]
row4 = [0, 0, 0, 0, 0, 0, 0, 0]
row5 = [0, 0, 0, 0, 0, 0, 0, 0]
row6 = [0, 0, 0, 0, 0, 0, 0, 0]
row7 = [0, 0, 0, 0, 0, 0, 0, 0]
row8 = [0, 0, 0, 0, 0, 0, 0, 0]


count = 0

data = np.zeros((8, 8))
sArray = np.zeros(300)
s = serial.Serial("/dev/ttyACM0", 115200)


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


def calc_s():
    global data
    # first haar-like filter
    left = np.hsplit(data, 2)[0]
    right = np.hsplit(data, 2)[1]
    sumLeft = np.sum(left, dtype=np.float64)
    sumRight = np.sum(right, dtype=np.float64)
    s1 = abs((sumLeft / 32) - (sumRight / 32))

    # second haar-like filter
    top = np.vsplit(data, 2)[0]
    bottom = np.vsplit(data, 2)[1]
    sumTop = np.sum(top, dtype=np.float64)
    sumBottom = np.sum(bottom, dtype=np.float64)
    s2 = abs((sumTop / 32) - (sumBottom / 32))

    # third haar like filter
    split1 = np.hsplit(data, 4)[0]
    split2 = np.hsplit(data, 4)[1]
    split3 = np.hsplit(data, 4)[2]
    split4 = np.hsplit(data, 4)[3]
    sum1 = np.sum(split1, dtype=np.float64)
    sum2 = np.sum(split2, dtype=np.float64) + np.sum(split3, dtype=np.float64)
    sum3 = np.sum(split4, dtype=np.float64)
    s3 = abs((2 * sum2 / 32) - (sum1 / 16) - (sum3 / 16))

    # final calc
    s = (s1 + s2 + s3) / 3
    print("S1: ", s1)
    print("S2: ", s2)
    print("S3: ", s3)
    return s


def append_window():

    global sArray
    sArray = np.roll(sArray, -1, axis=1)


for i in range(300):
    try:
        for j in range(8):

            read_loop()
            fill_data()

        sK = calc_s()
        sArray[i] = sK
        print(i)
    except Exception as e:
        print("Exception ", e)
while True:
    try:
        # filter now
        fs = 10
        nyq = 0.5 * fs
        normalCutoff = 0.1*math.pi
        b, a = butter(5, normalCutoff, btype='low', analog='False')
        y = lfilter(b, a, sArray)
        decSig = decimate(y, 10)

        # pylab.figure()
        # pylab.plot(decSig)
        maxima = argrelextrema(decSig, np.greater)
        print("Number of breaths: ", np.size(maxima))
        # pylab.show()
        # roll matrix
        sArray = np.roll(sArray, -1)
        for j in range(8):
            read_loop()
            fill_data()
        sK = calc_s()
        sArray[299] = sK
    except Exception as e:
        exception_type, exception_object, exception_traceback = sys.exc_info()

        filename = exception_traceback.tb_frame.f_code.co_filename

        line_number = exception_traceback.tb_lineno


        print("Exception type: ", exception_type)

        print("File name: ", filename)

        print("Line number: ", line_number)