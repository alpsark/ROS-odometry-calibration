# ROS-odometry-calibration
odemetry cabibration for robotino robot using the article

cd ros_ws   
sudo apt-get install ros-kinetic-joy  
run "catkin_make" make sure there are no errors.  
edit ~/.bashrc, add "source /home/..../rosws/devel/setup.bash" to the end of the .bashrc file.  
close the terminal and reopen the terminal  
run roscore  
run vrep, open scenes/assignment3_robotino_sarkisla.ttt  
. ~/ros-ws/devel/setup.bash  
rosrun robotino_odometry robotino_odometry  

you can edit to cpp to see original calibrated file  

run vrep, open scenes/assignment3_calibrated.ttt
