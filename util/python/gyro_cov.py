import numpy as np
import matplotlib.pyplot as plt
import fileinput, calibration

tmp = []
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
    if vals[0] == "MPU9250Data":
        gyro_sf = 500.0 * 0.01745329251 / 32767.0
        tmp.append([float(vals[4]) * gyro_sf, float(vals[5]) * gyro_sf, float(vals[6]) * gyro_sf])

raw_data = np.array(tmp)
tmp = []

cov = np.cov(raw_data.T)

print(cov)