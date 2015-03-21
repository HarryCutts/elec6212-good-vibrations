from __future__ import division, print_function

import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
import time

BAUD_RATE = 1000000

MEASUREMENT_SIZE = 2

HOST_READY = 'r'
HEADER_DATA_SIZE = ord('s')
BUFFER_START = ord('b')

port = sys.argv[1]

def send_ready(ser):
    ser.write(bytes(HOST_READY, 'ascii'))

def read_data_size_header(ser):
    data = bytes(ser.read(size=4))
    assert data[0] == HEADER_DATA_SIZE

    measurements_per_buffer = data[1] | data[2] << 8
    num_inputs = data[3]

    return num_inputs, measurements_per_buffer

def read_block(ser, block_size):
    print("Waiting for block header...")
    header = bytes(ser.read(size=9+block_size[0]))
    assert header[0] == BUFFER_START
    # (Throw away the times)
    order = (header[9], header[10], header[11], header[12])

    num_bytes_to_read = block_size[0] * block_size[1] * MEASUREMENT_SIZE
    print("Waiting for block body...")
    data = bytes(ser.read(size=num_bytes_to_read))
    return (data, order)

ser = serial.Serial(baudrate=BAUD_RATE)

try:
    ser.setPort(port)
    ser.open()

    send_ready(ser)
    block_size = read_data_size_header(ser)
    print("Number of inputs and buffer size:", block_size)

    time.sleep(0.1)

    d = np.array([])
    while True:
        while ser.inWaiting() == 0:
            pass

        block = read_block(ser, block_size)
        print("got")
        data_array = np.fromstring(block[0], dtype='<i2').reshape(block_size[1],block_size[0]).swapaxes(0,1)
            
        plt.clf()

        plt.plot(data_array.swapaxes(0,1));
        plt.show();

        means = data_array.mean(axis=1)
        rangeR = data_array.ptp(axis=1)

        # print(means)
        print(block[1])
        # print(means[:, np.newaxis])

        meanC = data_array - means[:, np.newaxis];
        norm = meanC / rangeR[:, np.newaxis];

        d = np.diff(norm, axis=1)
        a = np.absolute(d);

        index = 0;
        for x in a[0]:
            if x>0.1:
                print(index);
                break;
            index = index + 1;

        index1 = 0;
        for x in a[1]:
            if x>0.1:
                print(index1);
                break;
            index1 = index1 + 1;

        index2 = 0;
        for x in a[2]:
            if x>0.1:
                print(index2);
                break;
            index2 = index2 + 1;

        index3 = 0;
        for x in a[3]:
            if x>0.10:
                print(index3);
                break;
            index3 = index3 + 1;


        start = min([index,index1,index2,index3]) - 100
        print(start)

        plt.clf()
        plt.plot(a[0][start:]);
        plt.plot(a[1][start:]);
        plt.plot(a[2][start:]);
        plt.plot(a[3][start:]);
        plt.show();

        for x in range(4):
            for y in range(4):
                if x!=y:
                    corrData = plt.xcorr(data_array[x][start:],data_array[y][start:],normed=False);
                    maxLag = corrData[0][np.argmax(corrData[1])]
                    print(str(x) + " " + str(y) + " -> " + str(maxLag));

        import math
        first = plt.xcorr(data_array[3][start:],data_array[1][start:],normed=False);
        firstmaxLag = first[0][np.argmax(first[1])]
        second = plt.xcorr(data_array[2][start:],data_array[0][start:],normed=False);
        secondmaxLag = second[0][np.argmax(second[1])]


        print(math.atan2(firstmaxLag,secondmaxLag) * (180/math.pi))

finally:
    ser.close()
