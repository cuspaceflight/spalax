import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import scipy, math, fileinput

def down_sample(b, R):
    pad_size = math.ceil(float(b.shape[0]) / R) * R - b.shape[0]
    b_padded = np.append(b, np.zeros((pad_size, b.shape[1])) * np.NaN)
    temp = b_padded.reshape((-1, R, b.shape[1]))
    return scipy.nanmean(temp, axis=1)


def plot_scatter(data, name, down_sample_factor = 100):
    down_data = down_sample(data, down_sample_factor)
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(down_data[:, 0], down_data[:, 1], down_data[:, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(name)

def calibration_basic(data):
    min_magno = np.array([float("inf"), float("inf"), float("inf")])
    max_magno= -np.array([float("inf"), float("inf"), float("inf")])
    for vec in data:
        min_magno = np.minimum(min_magno, vec)
        max_magno = np.maximum(max_magno, vec)

    magno_sf = 1 / (max_magno - min_magno)
    magno_bias = (max_magno + min_magno) / 2

    return magno_sf, magno_bias


tmp = []
for line in fileinput.input():
    vals = line.strip().split(",")
    if vals[0] == "MPU9250Data":
        tmp.append([float(vals[7]), float(vals[8]), float(vals[9])])

raw_data = np.array(tmp)
tmp = []

plot_scatter(raw_data, "Raw Magno Data")
magno_sf, magno_bias = calibration_basic(raw_data)
plot_scatter((raw_data - magno_bias) * magno_sf, "Basic Calibration")

plt.show()

