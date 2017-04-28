from sklearn.preprocessing import normalize

import calibration
import fileinput

import matplotlib.pyplot as plt
import numpy as np

format_descriptors = False
tmp_accel = []
tmp_gyro = []
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
        tmp_accel.append([float(vals[1]), float(vals[2]), float(vals[3])])
        tmp_gyro.append([float(vals[4]), float(vals[5]), float(vals[6])])


accel_calibration_matrix = np.matrix([[3.34132659e-03, -3.04054733e-05, 1.38907583e-05],
                                      [-3.04054733e-05, 3.34291600e-03, -6.09264084e-06],
                                      [1.38907583e-05, -6.09264084e-06, 3.31029391e-03]])
accel_offset = np.array([5.59740051, -9.31069979,  3.52833252]).reshape((3,1))

tmp_calibrated_accel = []
tmp_calibrated_gyro = []

for accel in tmp_accel:
    raw = np.array(accel).reshape((3,1))
    offset = raw - accel_offset
    tmp_calibrated_accel.append(- 9.80665 * accel_calibration_matrix * offset)

for gyro in tmp_gyro:
    raw = np.array(gyro).reshape(3,1)
    tmp_calibrated_gyro.append(0.05*3.14159265359/180.0 * raw)

calibrated_accel = np.array(tmp_calibrated_accel)
calibrated_gyro = np.array(tmp_calibrated_gyro)

accel_mean = np.mean(calibrated_accel, axis=0)
gyro_mean = np.mean(calibrated_gyro, axis=0)

for accel in calibrated_accel:
    accel -= accel_mean

for gyro in calibrated_gyro:
    gyro -= gyro_mean


n = len(calibrated_gyro)
assert(len(calibrated_gyro) == len(calibrated_accel))
# Is there a relationship between the acceleration vector and the gyroscope vector?

angle = np.ones(len(calibrated_gyro))
i =0
for accel, gyro in zip(calibrated_accel, calibrated_gyro):
    accel_norm = np.linalg.norm(accel)
    gyro_norm = np.linalg.norm(gyro)

    if accel_norm < 1 or gyro_norm < 1:
        continue

    naccel = accel / accel_norm
    ngyro = gyro / gyro_norm

    angle[i] = np.dot(naccel.T, ngyro)
    i += 1

#plt.plot(angle[:i], label="Angle Plot")

i=0
exp_avg = np.array([0,0,0]).reshape(3,1)
accel_exp_avg = np.ones((len(calibrated_gyro), 3, 1))
for accel in calibrated_accel:
    alpha = 0.1
    exp_avg = (1-alpha)*exp_avg+alpha*accel
    accel_exp_avg[i] = exp_avg
    i += 1



#plt.plot(-10*np.diff(calibrated_accel, axis=0)[:,0], label="Jerk X")
#plt.plot(calibrated_gyro[:,1], label="Gyro Z")

#plt.plot(2*np.diff(calibrated_accel, axis=0)[:,2], label="Jerk Z")
#plt.plot(calibrated_gyro[:,0], label="Gyro X")

plt.plot(-4*np.diff(calibrated_accel, axis=0)[:,1], label="Jerk Y")
plt.plot(calibrated_gyro[:,1], label="Gyro Y")

#plt.plot(calibrated_accel[:,0], label="Accel X")
#plt.plot(accel_exp_avg[:,0], label="Accel Exp X")


plt.legend()
plt.grid()
plt.show()

