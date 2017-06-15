import scipy
from scipy import signal

import numpy as np
import matplotlib.pyplot as plt
import allantools
import math

fs = 0.5
T = 100
N = int(fs * T)
data = np.append(np.ones(N/2), np.zeros(N/2))

n = len(data)

plt.plot(np.fft.fftfreq(N, 1./fs), np.absolute(np.fft.fft(data)))

plt.show()