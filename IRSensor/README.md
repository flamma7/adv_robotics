# Magic 8 IR Sensor Calibration

By: Brandon Aguirre

There are 3 python scripts needed for the calibration process

+ pololuController.py
+ ir_measurements.py
+ plot_ir_data.py

## pololuController.py

This script contains a small classI wrote which is a controller for the pololu. It only has basic functionality to get input from the pololu.

## ir_measurements.py

This script is a command line tool I wrote for running a calibration on our IR sensors.

An overview of this process:

Run the script with `python3 ./ir_measurements.py` 

The script will run and count down 5 (default --sleep) seconds between each snapshot of measurements. The script will take 100 (default --n) measurements. It will repeat this process of counting down and taking a snapshot of measurements for 3 feet to 10 feet (default --range) by increments of 1 foot (default --increment). The script will write out to a file the data it collected as noted below.

output
```
distance_1,avg(measurements_1)
distance_2,avg(measurements_2)
distance_N,avg(measurements_N)
```

```
usage: ir_measurements.py [-h] [--out OUT] [--sleep SLEEP]
                          [--range RANGE RANGE] [--increment INCREMENT]
                          [--n N] [--channel CHANNEL] [--simulate]
                          [--sim_const SIM_CONST SIM_CONST SIM_CONST SIM_CONST]

optional arguments:
  -h, --help            show this help message and exit
  --out OUT             name of the file to write to
  --sleep SLEEP         integer for seconds to sleep between measurements
  --range RANGE RANGE   two integers specifying the range (inclusive) of feet
                        to measure
  --increment INCREMENT
                        integer specifying how much to increment between
                        measurements
  --n N                 integer specifying the number of measurements to take
                        at each increment
  --channel CHANNEL     integer specifying the channel the sensor is on
  --simulate            measurements will be simulated (i.e., no measurements
                        will be taken from device)
  --sim_const SIM_CONST SIM_CONST SIM_CONST SIM_CONST
                        four floating point numbers (slope, intercept, std
                        err, K) used to simulate measurements from IR sensor.
                        (must also use flag --simulate to use these constants)
```

## plot_ir_data.py

This script will plot the data created by the ir_measurements script, fit a regression model to the data, and output the plot. The script will also save the regression model to a file as a npy file. Read this stored data using `numpy.load('name_of_file_linregress.npy')`. The result will be an array containing `[slope, intercept, r_value, p_value, std_err]`. These values should be used to then predict the distance of the IR sensor to an object given the input of the IR Sensor. We do this with the following formulas.

```
V = Voltage (or the Analog to Digital signal)
R = Range
K = tuning constant

V = (1/(R + K)) : Linearization of sensor output vs range

We fit a line to the data and get the slope m and the incercept b

y = mx + b

y = (1/(R + K))
x = V

substitute y and x into the equation for a line

(1/(R + K)) = mV + b

Final equation used to estimate the range: R = (1/(mV + b)) - k
```

To see the plot of our sensors data look in the data dir `sensor_1_fig.png`.

```
usage: plot_ir_data.py [-h] [-k K] directory input_file sensor_name

positional arguments:
  directory    directory to save data in
  input_file   name of the file to read from
  sensor_name  name of the sensor which the data belongs to

optional arguments:
  -h, --help   show this help message and exit
  -k K

```
