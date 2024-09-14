# Mobile-Robot-Control

### Connect to Rosserial and Control Mobile Robot

```bash
# Start roscore
roscore

# Change permissions for the device
sudo chmod a+rw /dev/ttyUSB0

# Check tty devices
dmesg | grep tty

# Run rosserial
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600

Control the mobile robot: Use dat_circle.py (Press Ctrl+C to stop the robot)
Position the mobile robot:
Using ArUco Marker: server_odom.py
Using dead reckoning: pose.py
