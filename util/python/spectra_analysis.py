from scipy import signal

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
    if vals[0] == "ADIS16405Data":
        tmp.append(float(vals[7]))

data = np.array(tmp)

n = len(data)
fs = 1000.

f, Pxx_den = signal.periodogram(data, fs)
plt.semilogy(f, Pxx_den)
plt.ylim([1e-7, 1e2])
plt.xlabel('frequency [Hz]')
plt.ylabel('PSD [V**2/Hz]')
plt.show()