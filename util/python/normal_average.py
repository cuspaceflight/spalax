import scipy
from scipy import signal

import numpy as np
import matplotlib.pyplot as plt
import allantools
import fileinput
import math

from scipy.interpolate import UnivariateSpline

adis_accel_calibration_matrix = np.matrix([[3.34132659e-03, -3.04054733e-05, 1.38907583e-05],
                                           [-3.04054733e-05, 3.34291600e-03, -6.09264084e-06],
                                           [1.38907583e-05, -6.09264084e-06, 3.31029391e-03]])
adis_accel_offset = np.array([5.59740051, -9.31069979, 3.52833252]).reshape((3, 1))

adis_gyro_sf = 0.05 * math.pi / 180
mpu_gyro_sf = 500.0 * 0.01745329251 / 32767.0 * 180.0 / math.pi

format_descriptors = False
data_adis = []

adis_samples = 0

last_adis_timestamp = 0

timestamps = []

sensor = "magno"

for line in fileinput.input():
    vals = line.strip().split(",")
    if vals[0] == "StartFormatDescriptors":
        format_descriptors = True
        continue
    if format_descriptors:
        if vals[0] == "EndFormatDescriptors":
            format_descriptors = False
        else:
            continue
    if vals[0] == "ADIS16405Data":
        adis_samples += 1
        if sensor == 'gyro':
            data_adis.append(
                [float(vals[4]) * adis_gyro_sf, float(vals[5]) * adis_gyro_sf, float(vals[6]) * adis_gyro_sf])
        elif sensor == 'accel':
            data_adis.append(
                [float(vals[1]) * 3.34132659e-03, float(vals[2]) * 3.34132659e-03, float(vals[3]) * 3.34132659e-03])
        elif sensor == 'magno':
            data_adis.append(
                [float(vals[7]) * 1.07324296e-03, float(vals[8]) * 1.07324296e-03, float(vals[9]) * 1.07324296e-03])
        last_adis_timestamp = float(vals[11])
        timestamps.append(last_adis_timestamp)

data_adis = np.array(data_adis)
timestamps = np.array(timestamps)

average = np.average(data_adis, axis=0)

data_adis -= average

print(average)

print(scipy.stats.skew(data_adis))

print("Processing Data")

plt.plot(timestamps, np.random.normal(0, math.sqrt(5.12e-5), len(data_adis[:,0])), label="Normal Distribution With Expected Variance")

plt.plot(timestamps, data_adis[:,0], label="ADIS Gyro X Offset By Sample Average")



plt.plot(timestamps, np.random.normal(0, np.var(data_adis[:,0]), len(data_adis[:,0])), label="Normal Distribution With Sample Variance")



plt.legend()

plt.ylabel("Angular Velocity (rad/s)")
plt.xlabel("Time (s)")


plt.figure()
plt.hist(np.random.normal(np.average(data_adis[:,0]), math.sqrt(5.12e-5), len(data_adis[:,0])), bins=100)
plt.hist(data_adis[:,0], bins=100)
plt.grid()
plt.xlabel("Angular Velocity (rad/s)")
plt.ylabel("Frequency")


plt.show()

