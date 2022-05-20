import serial
import time
import threading
import readchar
import json
import tkinter as tk
import numpy as np
import sys
import seaborn as sb
import matplotlib as mpl
import matplotlib.pyplot as plt

row1 = [0, 0, 0, 0, 0, 0, 0, 0]
row2 = [0, 0, 0, 0, 0, 0, 0, 0]
row3 = [0, 0, 0, 0, 0, 0, 0, 0]
row4 = [0, 0, 0, 0, 0, 0, 0, 0]
row5 = [0, 0, 0, 0, 0, 0, 0, 0]
row6 = [0, 0, 0, 0, 0, 0, 0, 0]
row7 = [0, 0, 0, 0, 0, 0, 0, 0]
row8 = [0, 0, 0, 0, 0, 0, 0, 0]
plt.ion()
data = np.zeros((8,8))
fig = plt.imshow(data, cmap=plt.cm.hot,interpolation='lanczos')
plt.colorbar()
plt.clim(1,8)
plt.show()

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
    
    #ax = sb.heatmap(data, vmin=0, vmax=5, cmap="coolwarm")
    #fig.canvas.draw()
    #fig.canvas.flush_events()
    #plt.pause(0.0001)
    #plt.clf()

    fig.set_data(data)
    plt.show()
    

    print(data)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")



def read_loop():
    s = serial.Serial("/dev/ttyACM0", 115200)
    output = ""
    
    outArr = []

    while True:
        try:
            
            data = s.readline()
            #print(data)
            data = data.decode()
            #print(str(data))
            line = data.split(',')
            #print(line)
            line = line[:-1]
            index = line[0]
            del line[0]
            
            for i in range (8):
                if index == '0':
                    row1[i] = int(line[i])
                elif index == '1':
                    row2[i] = int(line[i])
                elif index == '2':
                    row3[i] = int(line[i])
                elif index == '3':
                    row4[i] = int(line[i])
                elif index == '4':
                    row5[i] = int(line[i])
                elif index == '5':
                    row6[i] = int(line[i])
                elif index == '6':
                    row7[i] = int(line[i])
                elif index == '7':
                    row8[i] = int(line[i])

            
            #heatmap()
            #print_grid()
            
            
            

        except Exception as e:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            print("Exception", e, "\r")
            print("Line", exception_traceback.tb_lineno)
            outArr = []
            NodeData = []
            output = ""
            continue
#if __name__ == "__main__":

    #t1 = threading.Thread(target=read_loop, args=())
    #t1.start()
    #t2 = threading.Thread(target=heatmap, args=())
    ##t2.start()
    #read_loop()
    #heatmap()