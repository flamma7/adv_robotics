import argparse
import time
import numpy as np
from pololuController import *


def count_down(seconds):
	"""
	This function sleeps for the given number of senconds while printing
	a count down to the screen
	@param seconds int: number of seconds to sleep
	"""
	if not seconds:
		return
	while seconds > 0:
		print(str(seconds) + '\r', end='')
		seconds -= 1
		time.sleep(1)
	print(' \r', end='')

def simulate(y, m, b, std_err, K):
	"""
	Simulate sensor data by generating an estimation+err "Analog to Digital" floating point number.
	The formula used is based on y = mx+b rearanged to estimate x.
	The y term is linearized
	@param y float distance from sensor
	@param m float slope of lin regression on real sensor data
	@param b float intercept of lin regression on real sensor data
	@param std_err float standard error of lin regression on real sensor data
	@param K float const recomended by https://acroname.com/articles/linearizing-sharp-ranger-data
	"""
	return ((1/(np.random.normal(y, std_err)+K)) - b) / m

def arange(a, b, inc=1):
	"""
	Generate an array of integers from (a, b) with increment of inc
	@param a int start of range (inclusive)
	@param b int end of range (inclusive)
	@param inc float
	"""
	x = [a]
	i = 0
	while x[i]+inc <= b:
		x.append(x[i]+inc)
		i += 1
	return x

def main():
	# read in arguments
	parser = argparse.ArgumentParser()
	parser.add_argument('--out', type=str,  default='measurements.txt', help='name of the file to write to')
	parser.add_argument('--sleep', type=int, default=5, help='integer for seconds to sleep between measurements')
	parser.add_argument('--range', type=int, default=[3, 10], nargs=2, help='two integers specifying the range (inclusive) of feet to measure')
	parser.add_argument('--increment', type=float, default=1, help='integer specifying how much to increment between measurements')
	parser.add_argument('--n', type=int, default=100, help='integer specifying the number of measurements to take at each increment')
	parser.add_argument('--channel', type=int, default=0, help='integer specifying the channel the sensor is on')
	parser.add_argument('--simulate', action='store_true', default=False, help='measurements will be simulated (i.e., no measurements will be taken from device)')
	parser.add_argument('--sim_const', nargs=4, help='four floating point numbers (slope, intercept, std err, K) used to simulate measurements from IR sensor. (must also use flag --simulate to use these constants)')
	args = parser.parse_args()

	sim_const = [0.0008073863884277423, -0.20900036625160207, 1.2076321721688548e-05, 0.43]
	maestro = None
	measurements = {}

	# adjust simulation constants if provided by user
	if args.sim_const:
		for i in range(len(args.sim_const)):
			sim_const[i] = float(args.sim_const[i])

	# define maestro controller if not a simulation
	if not args.simulate: maestro = PololuController()

	# loop through measurement units (feet)
	for feet in arange(args.range[0], args.range[1], args.increment):
		if args.simulate:
			print('Simulating measurements at {} feet'.format(feet))
		else:
			print('Measuring at {} feet'.format(feet))
		# count down to give user enough time to set up next measurement
		count_down(args.sleep)
		# create array for new entry in measurements at the current unit
		measurements[feet] = []
		# take N measurements
		for i in range(0, args.n):
			if args.simulate:
				# simulate the measurement
				measurements[feet].append(simulate(feet, sim_const[0], sim_const[1], sim_const[2], sim_const[3]))
			else:
				# use pololu to take measurement
				measurements[feet].append(maestro.getPosition(channel=args.channel))
				
	# write out to file the measurements taken
	with open(args.out, 'w') as file:
		for key, measurement in measurements.items():
			file.write('{},{}\n'.format(key, np.mean(measurement)))

if __name__ == '__main__':
	main()