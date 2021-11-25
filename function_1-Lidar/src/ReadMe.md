# Information

This folder contains everything needed to make our project.

### catkin_ws
Workspace used to compile code and use it with ROS on the LIDAR

### MSOP (Main Data Stream Output Protocol)
Used to see what are the distance information sent by the LIDAR.

### DIFOP (Device Information Output Protocol)
Used to see the different information of the LIDAR, like rotation speed, IPV4 addresses of the source (LIDAR) and the destination (laptop/Jetson), etc...

### UCWP (User Configuration Write Protocol)
WARNING : using this script may break the LIDAR, so be careful while using it.
The user can modify some parameters of the sensor as needed.
