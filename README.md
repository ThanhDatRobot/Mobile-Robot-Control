# Mobile-Robot-Control

Connect to Rosserial:

roscore

sudo chmod a+rw /dev/ttyUSB0

dmesg | grep tty

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600

1. To control mobile robot: dat_circle.py (Ctrl+C to Stop Robot)
   
2. Position mobile robot use ArUco Marker: server_odom.py

3. Position mobile robot use dead_reckoning: pose.py
