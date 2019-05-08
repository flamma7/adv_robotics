import matplotlib.pyplot as plt
import numpy as np

def rampup(x, rtime, mn, mx, alpha=5, lower=.5):
    sx = alpha*(x - (lower*rtime))
    for i in sx:
        yield mn + (mx - mn) * (1 / (1 + np.exp(-i)))


rtime = 2
time = np.linspace(0, rtime, 100)

plt.plot(time, [y for y in rampup(time, rtime, 20, 100)])
plt.plot(time, [y for y in rampup(time, rtime, 20, 100, lower=.3)])
plt.plot(time, [y for y in rampup(time, rtime, 20, 100, lower=.7)])
plt.show()
