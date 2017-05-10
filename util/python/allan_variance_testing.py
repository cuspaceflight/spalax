from scipy import signal

import numpy as np
import matplotlib.pyplot as plt
import allantools
import fileinput
import math

taus = [tt for tt in np.logspace(-1, 4, 100)]


angle_random_walk = 0.0158489
bias_instability = 5.011872336e-3
sample_rate = 800

N = 1000000


data = np.random.normal(0, angle_random_walk * math.sqrt(sample_rate), N)

bias = 0
for i in range(N):
    data[i] += bias
    bias += np.random.normal(0, bias_instability / sample_rate)


(taus, devs_x, _, _) = allantools.oadev(data, rate=sample_rate, data_type='freq', taus=taus)
for (x,y) in zip(taus, devs_x):
    if (x > 1 and x < 1.1):
        print("Angle Random Walk")
        print((x,y))

print "Bias Instability"
print (np.min(devs_x))

plt.loglog(taus, devs_x)


plt.grid()
plt.show()
