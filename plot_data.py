import argparse

import matplotlib.pyplot as plt

import receiver

import math

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('port', type=str,
                    help="The location of the Arduino's Native USB port")
parser.add_argument('--num-blocks', type=int, default=1,
                    help="The number of blocks to acquire and plot")
args = parser.parse_args()


def scale_linear_bycolumn(rawpoints):
    mins = np.min(rawpoints, axis=1)
    maxs = np.max(rawpoints, axis=1)
    rng = maxs - mins
    return np.swapaxes(np.swapaxes(rawpoints, 0, 1) / (rng/2), 0, 1)


def plot_data(data):
    for mic in range(receiver.NUM_INPUTS):
        plt.plot(data[mic], label=str(mic+1))
    plt.legend(loc='upper center', shadow=True, fontsize='x-large')
    plt.show()

data = receiver.get_data(args.port, args.num_blocks)
print(data)
plot_data(data)

means = receiver._mean_of_flat(data)
data[0] = data[0] - means[0]
data[1] = data[1] - means[1]
data[2] = data[2] - means[2]
data[3] = data[3] - means[3]
data = scale_linear_bycolumn(data)
data = np.absolute(data)
print(data)
print(data.mean(axis=1))
plot_data(data)

fig = plt.figure()
ax1 = fig.add_subplot(311)
d1 = ax1.xcorr(data[3], data[1], usevlines=True, maxlags=50, normed=False, lw=2)
ax1.grid(True)
ax1.axhline(0, color='black', lw=2)
ax1 = fig.add_subplot(312)
d2 = ax1.xcorr(data[2], data[0], usevlines=True, maxlags=50, normed=False, lw=2)
ax1.grid(True)
ax1.axhline(0, color='black', lw=2)
ax1 = fig.add_subplot(313)
d3 = ax1.xcorr(data[1], data[3], usevlines=True, maxlags=50, normed=False, lw=2)
ax1.grid(True)
ax1.axhline(0, color='black', lw=2)
plt.show()

t1 = d1[0][np.argmax(np.absolute(d1[1]))]
t2 = d2[0][np.argmax(np.absolute(d2[1]))]
t3 = d3[0][np.argmax(np.absolute(d3[1]))]


print(t1)
print(t2)
print(t3)

if(abs(t1) >= abs(t2)):
    angle = math.atan2(t1, t2)
else:
    angle = math.atan2(t2, t3) - math.pi/2

print(math.degrees(angle))
