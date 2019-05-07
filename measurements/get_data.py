import numpy as np
import pickle
from os.path import join, isfile
import sys
from DataCube import DataCube

folder = './data'
data_loc = 'measurements.pkl'

def main():
  data = None
  if isfile(join(folder, data_loc)):
    print('loading data...')
    with open(join(folder, data_loc), 'rb') as file:
      data = pickle.load(file)
    print('data is loaded.')
  else:
    print('there is no data file at {}...'.format(join(folder, data_loc)))

  if len(sys.argv) != 4:
    print('usage: python3 record_data.py ramp_dist drive voltage')
  else:
    print('jump distance is ** {} ** inches at\n\tramp_dist = {},\n\tdrive = {},\n\tvoltage = {}'.format(
      data.gets([
        ('ramp_dist', float(sys.argv[1])),
        ('drive', float(sys.argv[2])),
        ('voltage', float(sys.argv[3]))
      ]), sys.argv[1], sys.argv[2], sys.argv[3])
    )

if __name__ == '__main__':
  main()