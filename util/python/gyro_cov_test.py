import scipy
from scipy import signal

import numpy as np
import matplotlib.pyplot as plt
import allantools
import fileinput
import math

array = np.empty(100)

for i in range(100):
    array[i] = np.average(np.random.normal(0, math.sqrt(5.12e-5), 5000))

plt.plot(array)
plt.show()

