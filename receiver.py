from __future__ import division, print_function

import argparse
import sys
import serial
import time

import numpy as np
import matplotlib.pyplot as plt

BAUD_RATE = 1000000
MEASUREMENTS_PER_BUFFER = 2048 * 4
NUM_INPUTS = 4

parser = argparse.ArgumentParser()
parser.add_argument('port', type=str,
        help="The location of the Arduino's Native USB port")
parser.add_argument('--num-blocks', type=int, default=2,
        help="The number of blocks to acquire and plot")
args = parser.parse_args()

port = args.port

ser = serial.Serial(baudrate=BAUD_RATE)

def combine_two_bytes(d):
    return d[0] | d[1] << 8


def order_microphones(block):
    """Returns the block with the microphones sorted by their averages."""
    averages = np.mean(block, axis=1)
    return block[np.argsort(averages)]


def read_block(ser):
    block = [[] for i in range(NUM_INPUTS)]
    for y in range(MEASUREMENTS_PER_BUFFER):
        for mic in range(NUM_INPUTS):
            num = combine_two_bytes(ser.read(size=2))
            block[mic].append(num)

    return order_microphones(np.array(block))


def get_data(num_blocks):
    try:
        ser.setPort(port)
        ser.open()

        data = None

        for block_number in range(num_blocks):
            print("Next block")
            block = read_block(ser)
            data = np.concatenate((data, block), axis=1) if data is not None else block

        return data

    finally:
        ser.close()


def plot_data(data):
    for mic in range(NUM_INPUTS):
        plt.plot(data[mic])

    plt.show()


plot_data(get_data(args.num_blocks))
