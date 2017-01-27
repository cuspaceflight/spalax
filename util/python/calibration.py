import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import scipy, math
from ellipsoid_fit import ellipsoid_fit as ellipsoid_fit, data_regularize

def down_sample(b, R):
    pad_size = math.ceil(float(b.shape[0]) / R) * R - b.shape[0]
    b_padded = np.append(b, np.zeros((int(pad_size), b.shape[1])) * np.NaN)
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

    calibrated = (data - magno_bias) * magno_sf
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

    print("Ellipsoid Calibration (Transform - {}, Offset - {} Eval - {})".format(
        np.array_str(transform),
        np.array_str(center.T),
        str(eval)))
    return transform

def calibration_combination(data):
    min_magno = np.array([float("inf"), float("inf"), float("inf")])
    max_magno = -np.array([float("inf"), float("inf"), float("inf")])
    for vec in data:
        min_magno = np.minimum(min_magno, vec)
        max_magno = np.maximum(max_magno, vec)

    magno_sf = 2 / (max_magno - min_magno)
    magno_bias = (max_magno + min_magno) / 2

    calibrated_initial = (data - magno_bias) * magno_sf

    data2 = data_regularize(calibrated_initial)

    center, (a, b, c), evecs, v = ellipsoid_fit(data2)
    D = np.array([[1 / a, 0., 0.], [0., 1 / b, 0.], [0., 0., 1 / c]])


    transform = np.dot(np.diag(magno_sf),evecs.dot(D).dot(evecs.T))
    offset = center.T + magno_bias

    calibrated = transform.dot((data - offset).T).T
    plot_scatter(calibrated, "Combination Calibration", 100)
    eval = eval_calibrated(calibrated)
    print("Combination Calibration (Transform - {}, Offset - {} Eval - {})".format(
        np.array_str(transform),
        np.array_str(offset),
        str(eval)))

def calibration_all(data):
    plot_scatter(data, "Raw Magno Data")

    print ("Performing Basic Calibration")

    calibration_basic(data)

    print ("Performing Mean Calibration")

    calibration_basic_mean(data)

    print ("Performing Ellipsoid Calibration")

    calibration_ellipsoid(data)

    print ("Performing Combination Calibration")

    calibration_combination(data)