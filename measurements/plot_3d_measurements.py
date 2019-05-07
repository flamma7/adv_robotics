import pandas as pd
import numpy as np
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.colors import to_hex
from os.path import join, isfile
from DataCube import DataCube

from scipy import stats

shape = [
  (('ramp_dist', np.float64), { 'min': 5, 'max': 16, 'inc': 1 }),
  (('drive', int), { 'min': 40, 'max': 101, 'inc': 3 }),
  (('voltage', int), { 'min': 7.7, 'max': 8.11, 'inc': 0.01 }),
]

folder = './data'
data_loc = 'measurements.pkl'

data = None
if isfile(join(folder, data_loc)):
    print('loading data...')
    with open(join(folder, data_loc), 'rb') as file:
        data = pickle.load(file)
    print('data is loaded.')
else:
    print('creating data file...')
    data = DataTesseract(shape)
    with open(join(folder, data_loc), 'wb') as file:
        pickle.dump(data, file)
    print('data file has been created.')

# plot the values at those set above
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
volt = np.arange(shape[2][1]['min'], shape[2][1]['max'], shape[2][1]['inc'])
colors = [cm.viridis(x) for x in range(0, 400, 40)]
xs, ys, zs = [], [], []
volt_color_range = []

drive_11, y_11 = [], []
for vo in range(int((shape[2][1]['max'] - shape[2][1]['min']) / shape[2][1]['inc'])):
    for dr in range(shape[1][1]['min'], shape[1][1]['max']-1, shape[1][1]['inc']):
        for di in range(shape[0][1]['min'], shape[0][1]['max']-1, shape[0][1]['inc']):
            y = data.gets([('ramp_dist', di), ('drive', dr), ('voltage', float(np.round(volt[vo], 2)))])
            if y > 1:
                xs.append(di)
                ys.append(y)
                zs.append(dr)
                volt_color_range.append(float(np.round(volt[vo], 2)))
                if di == 11:
                    drive_11.append(dr)
                    y_11.append(y)

sc = ax.scatter(xs, ys, zs, c=volt_color_range, cmap=cm.viridis)

slope, intercept, r_value, p_value, std_err = stats.linregress(drive_11, y_11)
x = np.linspace(30, 80, 2)
y = [ slope*i + intercept for i in x]
z = [11]*2
ax.plot(z, y, x, c='r', label='plotted')

ax.set_xlabel('Ramp Distance')
ax.set_ylabel('Jump Distance')
ax.set_zlabel('Drive')
fig.colorbar(sc, label='Voltage')
# plt.title()
plt.legend()
plt.show()