from __future__ import division, print_function

import sys
import serial
import time

import numpy as np
import matplotlib.pyplot as plt

BAUD_RATE = 1000000
MEASUREMENTS_PER_BUFFER = 2048 * 4
NUM_INPUTS = 4

ser = serial.Serial(baudrate=BAUD_RATE)

def _combine_two_bytes(d):
    return d[0] | d[1] << 8


def _order_microphones(block):
    """Returns the block with the microphones sorted by their averages."""
    averages = np.mean(block, axis=1)
    return block[np.argsort(averages)]


def read_block(ser):
    block = [[] for i in range(NUM_INPUTS)]
    for y in range(MEASUREMENTS_PER_BUFFER):
        for mic in range(NUM_INPUTS):
            num = _combine_two_bytes(ser.read(size=2))
            block[mic].append(num)

    return _order_microphones(np.array(block))


def get_data(port, num_blocks):
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
