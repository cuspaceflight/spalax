import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import scipy, math, fileinput
from ellipsoid_fit import ellipsoid_fit as ellipsoid_fit, data_regularize


def down_sample(b, R):
    pad_size = math.ceil(float(b.shape[0]) / R) * R - b.shape[0]
    b_padded = np.append(b, np.zeros((pad_size, b.shape[1])) * np.NaN)
    temp = b_padded.reshape((-1, R, b.shape[1]))
    return scipy.nanmean(temp, axis=1)


def plot_scatter(data, name, down_sample_factor = 100):
    if down_sample_factor > 1:
        data = down_sample(data, down_sample_factor)
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(data[:, 0], data[:, 1], data[:, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(name)

def eval_calibrated(data):
    lengths = np.linalg.norm(data, axis=1)
    errors = lengths - 1
    errors = errors * errors
    return {'Mean Length': np.mean(lengths), 'Square Error Total': np.sum(errors), 'Average Error': np.mean(errors), 'Error standard deviation': np.std(errors)}

def calibration_basic(data):
    min_magno = np.array([float("inf"), float("inf"), float("inf")])
    max_magno= -np.array([float("inf"), float("inf"), float("inf")])
    for vec in data:
        min_magno = np.minimum(min_magno, vec)
        max_magno = np.maximum(max_magno, vec)

    magno_sf = 2 / (max_magno - min_magno)
    magno_bias = (max_magno + min_magno) / 2

    calibrated = (data - magno_bias) * magno_sf
    eval = eval_calibrated(calibrated)

    print("Basic Calibration (SF - {}, Bias - {}, Eval - {})".format(np.array_str(magno_sf), np.array_str(magno_bias), str(eval)))
    plot_scatter(calibrated, "Basic Calibration")

def calibration_basic_mean(data):
    min_magno = np.array([float("inf"), float("inf"), float("inf")])
    max_magno = -np.array([float("inf"), float("inf"), float("inf")])
    for vec in data:
        min_magno = np.minimum(min_magno, vec)
        max_magno = np.maximum(max_magno, vec)

    magno_bias = (max_magno + min_magno) / 2
    magno_sf = 1 / np.mean(np.linalg.norm(data - magno_bias, axis=1))

    calibrated = (raw_data - magno_bias) * magno_sf
    eval = eval_calibrated(calibrated)

    print(
        "Basic Calibration Mean SF (SF - {}, Bias - {}, Eval - {})".format(np.array_str(magno_sf), np.array_str(magno_bias),
                                                                  str(eval)))
    plot_scatter(calibrated, "Basic Calibration Mean SF")


def calibration_ellipsoid(data):
    data2 = data_regularize(data)

    center, (a, b, c), evecs, v = ellipsoid_fit(data2)
    D = np.array([[1 / a, 0., 0.], [0., 1 / b, 0.], [0., 0., 1 / c]])
    transform = evecs.dot(D).dot(evecs.T)


    calibrated = transform.dot((data - center.T).T).T
    plot_scatter(calibrated, "Ellipsoid Calibration", 100)
    eval = eval_calibrated(calibrated)

    print("Ellipsoid Calibration (Transform - {}, Eval - {})".format(np.array_str(transform),
                                                                    str(eval)))
    return transform

tmp = []
for line in fileinput.input():
    vals = line.strip().split(",")
    if vals[0] == "MPU9250Data":
        # Calibrate Accel
        # tmp.append([float(vals[1]), float(vals[2]), float(vals[3])])
        # Calibrate Magno
        tmp.append([float(vals[7]), float(vals[8]), float(vals[9])])

raw_data = np.array(tmp)
tmp = []

plot_scatter(raw_data, "Raw Magno Data")

calibration_basic(raw_data)
calibration_basic_mean(raw_data)
calibration_ellipsoid(raw_data)

plt.show()

