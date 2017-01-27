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
        tmp.append([float(vals[1]), float(vals[2]), float(vals[3])])

raw_data = np.array(tmp)
tmp = []

calibration.calibration_all(raw_data)

plt.show()
