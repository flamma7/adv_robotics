import numpy as np

class DataCube:
  def __init__(self, shape):
    if len(shape) != 3:
      raise Exception('shape should be of length 3, not {}'.format(len(shape)))
    self.data = np.zeros(
      shape=tuple(int((s[1]['max']-s[1]['min']) / s[1]['inc']) for s in shape)
    )
    
    self.shape = { i:{'name': shape[i][0][0],
              'type': shape[i][0][1],
              'min': shape[i][1]['min'],
              'max': shape[i][1]['max'], 
              'inc': shape[i][1]['inc']
           } for i in range(len(shape)) }
    
    self.imap = { shape[i][0][0]: i for i in range(len(shape))}
  
  def sets(self, setpoints, value):
    if len(setpoints) != 3:
      raise Exception('shape should be of length 3, not {}'.format(len(shape)))
    o = [0]*3
    for setpoint in setpoints:
      col = self.imap[setpoint[0]]
      if setpoint[1] < self.shape[col]['min'] or setpoint[1] > self.shape[col]['max']:
        raise Exception('setpoint "{}" with value {} is out of range [{}, {}]'
          .format(setpoint[0], setpoint[1], self.shape[col]['min'], self.shape[col]['max'])
        )
      o[col] = int((setpoint[1]-self.shape[col]['min']) / self.shape[col]['inc'])
    self.data[o[0]][o[1]][o[2]] = value
  
  def gets(self, setpoints):
    if len(setpoints) != 3:
      raise Exception('shape should be of length 3, not {}'.format(len(shape)))
    o = [0]*3
    for setpoint in setpoints:
      col = self.imap[setpoint[0]]
      if setpoint[1] < self.shape[col]['min'] or setpoint[1] > self.shape[col]['max']:
        raise Exception('setpoint "{}" with value {} is out of range [{}, {}]'
          .format(setpoint[0], setpoint[1], self.shape[col]['min'], self.shape[col]['max'])
        )
      o[col] = int((setpoint[1]-self.shape[col]['min']) / self.shape[col]['inc'])
    return self.data[o[0]][o[1]][o[2]]