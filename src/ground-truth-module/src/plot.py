import matplotlib.pyplot as plt
import csv
import numpy as np
import math
from matplotlib.ticker import FormatStrFormatter


def read_data(filename):

    data = []

    with open(filename, newline='') as csv_data:
        for row in csv_data:
            data.append([float(num_string.rstrip()) for num_string in row.split(sep = " ") if num_string != ''])

    return np.array(data)

def hist(axes, data, label_txt):
    axes.hist(data, label = label_txt)
    axes.legend()

def scatter(axes, data, label_txt):
    axes.plot(data, "o", markersize = 1, label = label_txt)
    axes.legend()

def main():

    # pred estimate
    estimate_data = read_data("./output.txt")
    tilt = estimate_data[:, 0] * 180.0 / math.pi
    vs = estimate_data[:, 1] * 180.0 / math.pi
    vg = estimate_data[:, 2] * 180.0 / math.pi
    ex = estimate_data[:, 17]
    ey = estimate_data[:, 18]
    ez = estimate_data[:, 19]
    nx = estimate_data[:, 20]
    ny = estimate_data[:, 21]
    nz = estimate_data[:, 22]
    etheta = estimate_data[:, 23] * 180.0 / math.pi

    # make plot
    fig, ax = plt.subplots(3, 4)
    hist(ax[0, 0], ex, "err x")
    hist(ax[0, 1], ey, "err y")
    hist(ax[0, 2], ez, "err z")
    hist(ax[0, 3], etheta, "error theta")

    scatter(ax[1, 0], ex, "err x")
    scatter(ax[1, 1], ey, "err y")
    scatter(ax[1, 2], ez, "err z")
    scatter(ax[1, 3], etheta, "err theta")

    scatter(ax[2, 0], nx, "n x")
    scatter(ax[2, 1], ny, "n y")
    scatter(ax[2, 2], nz, "n z")

    plt.show()

if __name__ == "__main__":
    main()
