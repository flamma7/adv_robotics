# adv_robotics
CSCI 4302 Class Project Repo. Autonomous wall guided car. 


## Setting up Bluetooth

```
git clone https://github.com/chrippa/ds4drv.git
cd ds4drv
sudo python setup.py install
sudo apt-get install ros-melodic-joy
ds4drv
```
Turn on the controller to pair
Once paired:
```
rosrun joy joy_node
```

magic_sm sets drive to 10 continuously

pid is the pid loop

driver control (not needed when teleop)

For teleop:
pololu controller
magic joy