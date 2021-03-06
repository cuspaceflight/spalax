import calibration
import fileinput

import matplotlib.pyplot as plt
import numpy as np

format_descriptors = False
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
    if vals[0] == "ADIS16405Data":
        tmp.append([float(vals[7]), float(vals[8]), float(vals[9])])

raw_data = np.array(tmp)
tmp = []

calibration.calibration_all(raw_data)

plt.show()
