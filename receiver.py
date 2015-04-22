from __future__ import division, print_function

import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
import time

BAUD_RATE = 1000000

port = sys.argv[1]

ser = serial.Serial(baudrate=BAUD_RATE)

totaldata = [[0] for i in range(4)]

mic = 0

try:
    ser.setPort(port)
    ser.open()

    for block_number in range(5):
        print("Next Block")
        for y in range(2048*4*4):
            while ser.inWaiting() == 0:
                pass

            d = ser.read(size=2)
            num = d[0] | d[1] << 8
            print(str(mic) + " -> " + str(num))

            totaldata[mic].append(num)

            mic = (mic + 1) % 4
        #print(np.array(data).mean(axis=1))

finally:
    ser.close()

plt.plot(totaldata[0])
plt.plot(totaldata[1])
plt.plot(totaldata[2])
plt.plot(totaldata[3])
plt.show()
