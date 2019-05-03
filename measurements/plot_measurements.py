import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def plot_measurements(msmnt):
  plt.plot(msmnt.speed, msmnt.jump_dist_inch,  marker='o')
  plt.title('Measurements: Speed vs Jump Dist.')
  plt.xlabel('speed')
  plt.ylabel('jump distance (inchs)')
  plt.show()

def main():
  fname = 'measurements.csv'
  msmnt = pd.read_csv(fname)
  plot_measurements(msmnt)

if __name__ == '__main__':
  main()