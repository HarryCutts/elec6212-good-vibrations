from __future__ import division, print_function

import sys
import serial
import time
import math

import numpy as np
import matplotlib.pyplot as plt

BAUD_RATE = 1000000
MEASUREMENTS_PER_BUFFER = 2048 * 4
NUM_INPUTS = 4

#breaking this indicates that the serial is corrupt
CORRUPT_MAX_THREASHOLD = 4000

#number of blocks to split the data into
STD_NUM_BLOCKS = 64
#defines if it is a flat part of the signal
STD_FLAT_THREASHOLD = 5
#max number of flat blocks before there isnt enough data to analyse
STD_MAX_FLAT_BLOCKS_PROPORTION = 0.75
STD_MAX_FLAT_BLOCKS = math.floor(STD_MAX_FLAT_BLOCKS_PROPORTION*STD_NUM_BLOCKS)

ser = serial.Serial(baudrate=BAUD_RATE)


def _combine_two_bytes(d):
    return d[0] | d[1] << 8


def _get_mean_std_blocks(block):
    """Returns an array of mean stds over all the signals.
    Useful for doing filtering and ordering"""
    mic_stds = [[] for i in range(NUM_INPUTS)]
    for mic in range(NUM_INPUTS):
        split_data = np.split(block[mic], STD_NUM_BLOCKS, axis=0)
        mic_stds[mic] = np.std(split_data, axis=1)
    mic_stds = np.array(mic_stds)
    return np.mean(mic_stds, axis=0)

def _last_flat_block(block):
    """Find the index of the last flat block"""
    mean_std = _get_mean_std_blocks(block)
    endIndex = 0
    for m in mean_std:
        if m < STD_FLAT_THREASHOLD:
            endIndex = endIndex + 1
        else:
            break
    print(endIndex)
    return endIndex

def _order_microphones(block):
    """Returns the block with the microphones
    sorted by their averages at their flattest point."""
    endIndex = _last_flat_block(block)
    averages = []
    for b in block:
        m = np.mean(b[0:endIndex*(MEASUREMENTS_PER_BUFFER/STD_NUM_BLOCKS)])
        averages.append(m)
    print(averages)

    averages = np.mean(block, axis=1)
    return block[np.argsort(averages)]


def _block_start_flat(block):
    """Finds if the first block is flat"""
    mean_std = _get_mean_std_blocks(block)
    return mean_std[0] < STD_FLAT_THREASHOLD


def _block_corrupted(block):
    """Finds corrupted serial comms.
    Data comes in as crazy large numbers with no patterns contained within"""
    maxes = np.max(block, axis=1)
    return max(maxes) > CORRUPT_MAX_THREASHOLD


def _block_too_flat(block):
    """Returns if the number of flat blocks is too many"""
    endIndex = _last_flat_block(block)
    return endIndex > STD_MAX_FLAT_BLOCKS


def read_block(ser, with_filter=True):
    block = [[] for i in range(NUM_INPUTS)]
    for y in range(MEASUREMENTS_PER_BUFFER):
        for mic in range(NUM_INPUTS):
            num = _combine_two_bytes(ser.read(size=2))
            block[mic].append(num)

    block = np.array(block)

    if with_filter:
        if _block_corrupted(block):
            print("Block corrupt")
            return None

        if not _block_start_flat(block):
            print("Not flat at start")
            return None

        if _block_too_flat(block):
            print("Too little signal due to too much flat")
            return None

    return _order_microphones(block)


def get_data(port, num_blocks, with_filter=True):
    try:
        ser.setPort(port)
        ser.open()

        data = None

        for block_number in range(num_blocks):
            print("Next block")
            block = read_block(ser, with_filter)
            while block is None:
                print("Bad block")
                block = read_block(ser, with_filter)
            data = np.concatenate((data, block), axis=1) if data is not None else block

        return data

    finally:
        ser.close()
