import numpy as np
import pickle
from os.path import join, isfile
import sys
from DataCube import DataCube

folder = './data'
data_loc = 'measurements.pkl'

shape = [
  (('ramp_dist', np.float64), { 'min': 5, 'max': 15, 'inc': 1 }),
  (('drive', int), { 'min': 40, 'max': 100, 'inc': 3 }),
  (('voltage', int), { 'min': 7.7, 'max': 8.5, 'inc': 0.01 }),
]

def save(location, data):
  with open(location, 'wb') as file:
    pickle.dump(data, file)

def main():
  data = None
  if isfile(join(folder, data_loc)):
    print('loading data...')
    with open(join(folder, data_loc), 'rb') as file:
      data = pickle.load(file)
    print('data is loaded.')
  else:
    print('creating data file...')
    data = DataCube(shape)
    with open(join(folder, data_loc), 'wb') as file:
      pickle.dump(data, file)
    print('data file has been created.')

  if len(sys.argv) != 5:
    print('usage: python3 record_data.py ramp_dist drive voltage jump_dist')
  else:
    data.sets([
      ('ramp_dist', float(sys.argv[1])),
      ('drive', float(sys.argv[2])),
      ('voltage', float(sys.argv[3]))
    ], float(sys.argv[4]))
    save(join(folder, data_loc), data)
    print('saved data:\n\tramp_dist = {},\n\tdrive = {},\n\tvoltage = {},\n\tjump_dist = {}'
      .format(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    )

if __name__ == '__main__':
  main()