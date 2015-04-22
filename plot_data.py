import argparse

import matplotlib.pyplot as plt

import receiver

parser = argparse.ArgumentParser()
parser.add_argument('port', type=str,
        help="The location of the Arduino's Native USB port")
parser.add_argument('--num-blocks', type=int, default=2,
        help="The number of blocks to acquire and plot")
args = parser.parse_args()


def plot_data(data):
    for mic in range(receiver.NUM_INPUTS):
        plt.plot(data[mic])

    plt.show()


plot_data(receiver.get_data(args.port, args.num_blocks))
