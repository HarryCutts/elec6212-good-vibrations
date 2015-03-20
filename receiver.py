from __future__ import division, print_function

import sys
import serial

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
    print("Waiting for header...")
    header = bytes(ser.read(size=9))
    assert header[0] == BUFFER_START
    # (Throw away the times)

    num_bytes_to_read = block_size[0] * block_size[1] * MEASUREMENT_SIZE
    print("Waiting for header...")
    data = bytes(ser.read(size=num_bytes_to_read))
    return data

ser = serial.Serial(baudrate=BAUD_RATE)

try:
    ser.setPort(port)
    ser.open()

    send_ready(ser)
    block_size = read_data_size_header(ser)
    print("Number of inputs and buffer size:", block_size)

    block = read_block(ser, block_size)
    print(block)


finally:
    ser.close()
