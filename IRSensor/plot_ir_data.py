import argparse
import matplotlib.pyplot as plt
import numpy as np
from os.path import join
from scipy import stats

def main():
	# read command line args
	parser = argparse.ArgumentParser()
	parser.add_argument('directory', type=str, help='directory to save data in')
	parser.add_argument('input_file', type=str, help='name of the file to read from')
	parser.add_argument('sensor_name', type=str, help='name of the sensor which the data belongs to')
	parser.add_argument('-k', type=float, help='')
	args = parser.parse_args()

	# set k if user set k
	k = args.k or .43

	file_suffix_numpy = '_linregress'
	file_suffix_fig = '_fig'

	x = []
	y = []

	# read in data from input file
	with open(join(args.directory, args.input_file), 'r') as file:
		for line in file:
			line = line.strip().split(',')
			x.append(int(float(line[1].strip())))
			y.append(int(float(line[0].strip())))

	# make x numpy array
	x = np.asarray(x)
	# linearize y: (1/(y+k))
	y_lin = np.divide(1, np.add(y, k))
	# perform linear regression
	slope, intercept, r_value, p_value, std_err = stats.linregress(x, y_lin)
	# save values to file
	np.save(join(args.directory, args.sensor_name+file_suffix_numpy), np.array([slope, intercept, r_value, p_value, std_err]))
	# plot
	plt.scatter(x, y_lin)
	plt.plot(x, intercept + x*slope, 'r')
	plt.title('Linearized Range vs. Analog to Digital')
	plt.ylabel('Linearized Range (1/(R+K))')
	plt.xlabel('Analog to Digital Conversion')
	# save figure to file
	plt.savefig(join(args.directory, args.sensor_name+file_suffix_fig))
	plt.show()

if __name__ == '__main__':
	main()