import pandas as pd
import numpy as np
import pickle
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import to_hex
from os.path import join, isfile
from DataCube import DataCube

from scipy import stats

def plot_measurements(msmnt):
  plt.plot(msmnt.speed, msmnt.jump_dist_inch,  marker='o')
  plt.title('Measurements: Speed vs Jump Dist.')
  plt.xlabel('speed')
  plt.ylabel('jump distance (inchs)')
  plt.show()

def main():
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



	# plot the values at those set above
	fig = plt.figure(figsize=(10, 8))
	ax = fig.add_subplot(111)
	volt = np.arange(shape[2][1]['min'], shape[2][1]['max'], shape[2][1]['inc'])
	colors = [cm.viridis(x) for x in range(0, 400, 40)]
	volt_color_range = []

	voi = int((shape[2][1]['max'] - shape[2][1]['min']) / shape[2][1]['inc'])
	dri = int((shape[1][1]['max'] - shape[1][1]['min']-1) / shape[1][1]['inc'])
	dii = int((shape[0][1]['max'] - shape[0][1]['min']-1) / shape[0][1]['inc'])
	
	drives = np.zeros((dii, dri))
	jumpdist = np.zeros((dii, dri))

	for vo in range(voi):
		for dr in range(dri):
			for di in range(dii):
				y = data.data[di][dr][vo]
				if y > 1:
					drives[di][dr-1] = (dr*shape[1][1]['inc']) + shape[1][1]['min']
					jumpdist[di][dr-1] = y

	axs = []
	ramp_incs = list(range(shape[0][1]['min'], shape[0][1]['max']-1, shape[0][1]['inc']))
	for i, dr, jd in zip(ramp_incs, drives, jumpdist):
		sc = ax.scatter(jd, dr, label='Ramp Up Dist {}'.format(i))
		axs.append(sc)

	# slope, intercept, r_value, p_value, std_err = stats.linregress(drive_11, y_11)
	# x = np.linspace(30, 80, 2)
	# y = [ slope*i + intercept for i in x]
	# z = [11]*2
	# ax.plot(z, y, x, c='r', label='plotted')

	ax.set_xlabel('Ramp Distance')
	ax.set_ylabel('Drive')
	# ax.set_zlabel('Drive')
	# fig.colorbar(sc, label='Voltage')
	# # plt.title()
	plt.legend()
	plt.show()

if __name__ == '__main__':
  main()