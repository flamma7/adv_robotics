
To set up the Phidget IMU on Ubuntu using melodic:

https://www.phidgets.com/docs/OS_-_Linux#Getting_started_with_Linux

Prereqs: libusb-1.0-0-dev

wget https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz

Unpack it and cd libphidget22-1.0.20190228 (your directory name might be different!!)  :

We need /usr/local/lib to be in the LD_LIBRARY_PATH environment variable. Check to see if that is the case:
printenv |grep LD_LIBRAR
if not, update this by using:
echo "/usr/local/lib" >> /etc/ld.so.conf && sudo ldconfig

If you get a permission denied, then just add

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

to your .bashrc and resource.

Then, still in the unpackled phidget directory:

./configure
make
make install     ensure no errors

#Update udev rules

sudo cp plat/linux/udev/99-libphidget22.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

Then follow the verification procedure at the website found at the top.

Usage:

the gcc compiler flag -lphidget22 needs to be added, as well as including the proper header file in src files:
#include <phidget22.h>

Now, for Python:

https://www.phidgets.com/docs/Language_-_Python_Linux_Terminal

