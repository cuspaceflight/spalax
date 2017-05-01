import numpy as np
import matplotlib.pyplot as plt
import allantools
import fileinput

accel_calibration_matrix = np.matrix([[3.34132659e-03, -3.04054733e-05, 1.38907583e-05],
                                      [-3.04054733e-05, 3.34291600e-03, -6.09264084e-06],
                                      [1.38907583e-05, -6.09264084e-06, 3.31029391e-03]])
accel_offset = np.array([5.59740051, -9.31069979,  3.52833252]).reshape((3,1))

gyro_sf = 500.0 * 0.01745329251 / 32767.0

format_descriptors = False
gyro_data = []
accel_data = []

mpu_samples = 0
adis_samples = 0

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
        gyro_data.append([float(vals[4]) * gyro_sf, float(vals[5]) * gyro_sf, float(vals[6]) * gyro_sf])

        accel = np.array([float(vals[1]), float(vals[2]), float(vals[3])])
        accel_data.append(accel_calibration_matrix * (accel - accel_offset))

    elif vals[0] == "MPU9250Data":
        mpu_samples += 1

gyro_data = np.array(gyro_data)#np.array(np.random.normal(0, 5, 100000))
accel_data = np.array(accel_data)


mpu_sample_rate = 1000.0
adis_sample_rate = mpu_sample_rate * adis_samples / mpu_samples

a = allantools.Dataset(data=accel_data[:, 1], rate=adis_sample_rate)
a.compute("oadev")

b = allantools.Plot()
b.plot(a, errorbars=True, grid=True)
b.ax.set_xlabel("Tau (s)")
b.show()