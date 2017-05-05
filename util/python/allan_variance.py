from scipy import signal

import numpy as np
import matplotlib.pyplot as plt
import allantools
import fileinput
import math

adis_accel_calibration_matrix = np.matrix([[3.34132659e-03, -3.04054733e-05, 1.38907583e-05],
                                           [-3.04054733e-05, 3.34291600e-03, -6.09264084e-06],
                                           [1.38907583e-05, -6.09264084e-06, 3.31029391e-03]])
adis_accel_offset = np.array([5.59740051, -9.31069979, 3.52833252]).reshape((3, 1))

adis_gyro_sf = 0.05
mpu_gyro_sf = 500.0 * 0.01745329251 / 32767.0 * 180.0 / math.pi

format_descriptors = False
data_adis = []
data_mpu = []

mpu_samples = 0
adis_samples = 0

last_mpu_timestamp = 0
last_adis_timestamp = 0

sensor = 'magno'
adis = False
mpu = True

psd = True
allan = False

if psd:
    plt.ylim([1e-14, 1e-3])


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

    elif vals[0] == "MPU9250Data":
        mpu_samples += 1
        if sensor == 'gyro':
            data_mpu.append([float(vals[4]) * mpu_gyro_sf, float(vals[5]) * mpu_gyro_sf, float(vals[6]) * mpu_gyro_sf])
        elif sensor == 'accel':
            data_mpu.append(
                [float(vals[1]) * 4.91349087e-04, float(vals[2]) * 4.91349087e-04, float(vals[3]) * 4.91349087e-04])
        elif sensor == 'magno':
            data_mpu.append(
                [float(vals[7]) * 3.61694852e-03, float(vals[8]) * 3.61694852e-03, float(vals[9]) * 3.61694852e-03])
        last_mpu_timestamp = float(vals[10])

data_adis = np.array(data_adis)
data_mpu = np.array(data_mpu)

print("Processing Data")

mpu_sample_rate = len(data_mpu) / last_mpu_timestamp
adis_sample_rate = len(data_adis) / last_adis_timestamp

print("Last MPU Timestamp: " + str(last_mpu_timestamp))
print("Last ADIS Timestamp: " + str(last_adis_timestamp))

print("MPU Sample Rate: " + str(mpu_sample_rate))
print("ADIS Sample Rate: " + str(adis_sample_rate))

taus = [tt for tt in np.logspace(-1, 4, 100)]

if (allan):

    if sensor == 'gyro':
        (taus, devs_x, _, _) = allantools.oadev(data_adis[:, 0], rate=adis_sample_rate, data_type='freq', taus=taus)
        (taus, devs_y, _, _) = allantools.oadev(data_adis[:, 1], rate=adis_sample_rate, data_type='freq', taus=taus)
        (taus, devs_z, _, _) = allantools.oadev(data_adis[:, 2], rate=adis_sample_rate, data_type='freq', taus=taus)

        plt.loglog(taus, devs_x, label='ADIS Gyro X')
        plt.loglog(taus, devs_y, label='ADIS Gyro Y')
        plt.loglog(taus, devs_z, label='ADIS Gyro Z')

        (taus, devs_x, _, _) = allantools.oadev(data_mpu[:, 0], rate=mpu_sample_rate, data_type='freq', taus=taus)
        (taus, devs_y, _, _) = allantools.oadev(data_mpu[:, 1], rate=mpu_sample_rate, data_type='freq', taus=taus)
        (taus, devs_z, _, _) = allantools.oadev(data_mpu[:, 2], rate=mpu_sample_rate, data_type='freq', taus=taus)

        plt.loglog(taus, devs_x, label='MPU Gyro X')
        plt.loglog(taus, devs_y, label='MPU Gyro Y')
        plt.loglog(taus, devs_z, label='MPU Gyro Z')

        plt.ylabel("Root Allan Variance ($^\circ$/ s)")
        plt.xlabel("Tau (sec)")
        plt.legend()
        plt.grid()


    elif sensor == 'accel':
        (taus, devs_x, _, _) = allantools.oadev(data_adis[:, 0], rate=adis_sample_rate, data_type='freq', taus=taus)
        (taus, devs_y, _, _) = allantools.oadev(data_adis[:, 1], rate=adis_sample_rate, data_type='freq', taus=taus)
        (taus, devs_z, _, _) = allantools.oadev(data_adis[:, 2], rate=adis_sample_rate, data_type='freq', taus=taus)

        plt.loglog(taus, devs_x, label='ADIS Accel X')
        plt.loglog(taus, devs_y, label='ADIS Accel Y')
        plt.loglog(taus, devs_z, label='ADIS Accel Z')

        (taus, devs_x, _, _) = allantools.oadev(data_mpu[:, 0], rate=mpu_sample_rate, data_type='freq', taus=taus)
        (taus, devs_y, _, _) = allantools.oadev(data_mpu[:, 1], rate=mpu_sample_rate, data_type='freq', taus=taus)
        (taus, devs_z, _, _) = allantools.oadev(data_mpu[:, 2], rate=mpu_sample_rate, data_type='freq', taus=taus)

        plt.loglog(taus, devs_x, label='MPU Accel X')
        plt.loglog(taus, devs_y, label='MPU Accel Y')
        plt.loglog(taus, devs_z, label='MPU Accel Z')

        plt.ylabel("Root Allan Variance (g)")
        plt.xlabel("Tau (sec)")
        plt.legend()
        plt.grid()

    elif sensor == 'magno':
        (taus, devs_x, _, _) = allantools.oadev(data_adis[:, 0], rate=adis_sample_rate, data_type='freq', taus=taus)
        (taus, devs_y, _, _) = allantools.oadev(data_adis[:, 1], rate=adis_sample_rate, data_type='freq', taus=taus)
        (taus, devs_z, _, _) = allantools.oadev(data_adis[:, 2], rate=adis_sample_rate, data_type='freq', taus=taus)

        plt.loglog(taus, devs_x, label='ADIS Magno X')
        plt.loglog(taus, devs_y, label='ADIS Magno Y')
        plt.loglog(taus, devs_z, label='ADIS Magno Z')

        (taus, devs_x, _, _) = allantools.oadev(data_mpu[:, 0], rate=mpu_sample_rate, data_type='freq', taus=taus)
        (taus, devs_y, _, _) = allantools.oadev(data_mpu[:, 1], rate=mpu_sample_rate, data_type='freq', taus=taus)
        (taus, devs_z, _, _) = allantools.oadev(data_mpu[:, 2], rate=mpu_sample_rate, data_type='freq', taus=taus)

        plt.loglog(taus, devs_x, label='MPU Magno X')
        plt.loglog(taus, devs_y, label='MPU Magno Y')
        plt.loglog(taus, devs_z, label='MPU Magno Z')

        plt.ylabel("Root Allan Variance (Normalised to Local Field Strength)")
        plt.xlabel("Tau (sec)")
        plt.legend()
        plt.grid()

elif psd:

    if sensor == 'gyro' and adis:
        f, Pxx_den = signal.periodogram(data_adis[:, 0], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Gyro X")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_adis[:, 1], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Gyro Y")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_adis[:, 2], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Gyro Z")
        plt.xlabel('frequency [Hz]')
    elif sensor == 'gyro' and mpu:
        f, Pxx_den = signal.periodogram(data_mpu[:, 0], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Gyro X")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_mpu[:, 1], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Gyro Y")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_mpu[:, 2], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Gyro Z")
        plt.xlabel('frequency [Hz]')
    elif sensor == 'accel' and adis:
        f, Pxx_den = signal.periodogram(data_adis[:, 0], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Accel X")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_adis[:, 1], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Accel Y")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_adis[:, 2], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Accel Z")
        plt.xlabel('frequency [Hz]')

    elif sensor == 'accel' and mpu:
        f, Pxx_den = signal.periodogram(data_mpu[:, 0], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Accel X")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_mpu[:, 1], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Accel Y")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_mpu[:, 2], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Accel Z")
        plt.xlabel('frequency [Hz]')
    elif sensor == 'magno' and adis:
        f, Pxx_den = signal.periodogram(data_adis[:, 0], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Magno X")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_adis[:, 1], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Magno Y")

        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_adis[:, 2], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="ADIS Magno Z")
        plt.xlabel('frequency [Hz]')

    elif sensor == 'magno' and mpu:
        f, Pxx_den = signal.periodogram(data_mpu[:, 0], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Magno X")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_mpu[:, 1], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Magno Y")
        plt.xlabel('frequency [Hz]')

        f, Pxx_den = signal.periodogram(data_mpu[:, 2], adis_sample_rate)
        plt.semilogy(f, Pxx_den, label="MPU Magno Z")
        plt.xlabel('frequency [Hz]')


plt.xlim([0, 300])
plt.grid()
plt.legend()
plt.show()
