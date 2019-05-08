#!/usr/bin/evn python

import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from os.path import join, isfile
from DataCube import DataCube
import pickle


folder = './data'
data_loc = 'measurements.pkl'

shape = [
    (('ramp_dist', np.float64), { 'min': 5, 'max': 16, 'inc': 1 }),
    (('drive', int), { 'min': 40, 'max': 101, 'inc': 3 }),
    (('voltage', int), { 'min': 7.7, 'max': 8.11, 'inc': 0.01 }),
]

data = None
if isfile(join(folder, data_loc)):
    print('loading data...')
    with open(join(folder, data_loc), 'rb') as file:
        data = pickle.load(file)
    print('data is loaded.')

voi = int((shape[2][1]['max'] - shape[2][1]['min']) / shape[2][1]['inc'])
dri = int((shape[1][1]['max'] - shape[1][1]['min']-1) / shape[1][1]['inc'])
dii = int((shape[0][1]['max'] - shape[0][1]['min']-1) / shape[0][1]['inc'])

drives = np.zeros((dii, dri))
jumpdist = np.zeros((dii, dri))

x = []
y = []
z = []

for vo in range(voi):
    for dr in range(dri):
        for di in range(dii):
            jd = data.data[di][dr][vo]
            if jd > 1:
                x.append((di*shape[0][1]['inc']) + shape[0][1]['min'])
                y.append(jd)
                z.append((dr*shape[1][1]['inc']) + shape[1][1]['min'])

# some 3-dim points
data = np.c_[x, y, z]

# regular grid covering the domain of the data
# regular grid covering the domain of the data
mn = np.min(data, axis=0)
mx = np.max(data, axis=0)
X,Y = np.meshgrid(np.linspace(mn[0], mx[0], 20), np.linspace(mn[1], mx[1], 20))
XX = X.flatten()
YY = Y.flatten()

order = 2    # 1: linear, 2: quadratic
if order == 1:
    # best-fit linear plane
    A = np.c_[data[:,0], data[:,1], np.ones(data.shape[0])]
    C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])    # coefficients
    
    # evaluate it on grid
    Z = C[0]*X + C[1]*Y + C[2]
    
    # or expressed using matrix/vector product
    #Z = np.dot(np.c_[XX, YY, np.ones(XX.shape)], C).reshape(X.shape)

elif order == 2:
    # best-fit quadratic curve
    A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
    C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])
    
    # evaluate it on a grid
    Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X.shape)

# plot points and fitted surface
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
ax.scatter(data[:,0], data[:,1], data[:,2], c='r', s=50)
plt.xlabel('Ramp Distance')
plt.ylabel('Jump Distance')
ax.set_zlabel('Drive Speed')
ax.axis('equal')
ax.axis('tight')
ax.set_zlim(30, 110)
plt.show()