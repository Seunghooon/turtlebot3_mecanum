# Turtlebot3_mecanum
###Turtlebot3_mecanum

1. Move turtlebot3_mecanum_core package to your Arduino folder.
 - Odometry information
   <img src="https://github.com/Seunghooon/turtlebot3_mecanum/blob/master/readme_images/turtlebot3_mecanum_odometry.png" width="350">

2. You can draw the map using 'mecanum_teleop_key' code in 'turtlebot3_teleop' package.

3. Before run 'roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml', you have to add 'export TURTLEBOT3_MODEL=mecanum' to the your bash file.

4. You can use the open source package 'follow_waypoints' to set the waypoints in the map.
 - Reference: [http://wiki.ros.org/follow_waypoints]




