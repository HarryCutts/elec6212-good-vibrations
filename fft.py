import argparse

import matplotlib.pyplot as plt

import receiver

import math

import numpy as np

from scipy import fftpack

import time

parser = argparse.ArgumentParser()
parser.add_argument('port', type=str,
                    help="The location of the Arduino's Native USB port")
parser.add_argument('--num-blocks', type=int, default=1,
                    help="The number of blocks to acquire and plot")
args = parser.parse_args()

plt.ion()
fig = plt.figure()
real_subplot = fig.add_subplot(1, 1, 1)
imaginary_subplot = fig.add_subplot(2, 1, 2)
real_graph = None
imaginary_graph = None
while True:
    data = receiver.get_data(args.port, 1, with_filter=False)
    means = np.mean(data, axis=1)

    print(means[0])
    fft = fftpack.fft(data[0] - means[0])

    fft_real = np.real(fft)
    fft_imaginary = np.imag(fft)

    if real_graph is not None:
        real_graph.set_ydata(fft_real[1:])
        imaginary_graph.set_ydata(fft_imaginary[1:])
    else:
        real_graph = real_subplot.plot(range(len(fft_real) - 1), fft_real[1:])[0]
        imaginary_graph = imaginary_subplot.plot(range(len(fft_imaginary) - 1), fft_imaginary[1:])[0]

    plt.draw()

