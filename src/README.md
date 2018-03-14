This repository represents the needed code to run the full navigation stack on the autonomous Taylor-Dunn SS-534. It was previously compiled on ROS Indigo

The "dunn" package is the master. Launch files contained within should call on other packages as needed.
    -dunn/params stores costmap parameters for the ROS navigation stack. These are the default parameters, and have not been modified to fit this vehicle in any way

"description" publishes the URDF data to describe and display a simplified model of the vehicle in RViz. Modifications should be made to car_test.xacro and then a new car_test.urdf should be generated via:
$rosrun xacro xacro.py car_test.xacro > car_test.urdf
car_test.gazebo should be complete, but has not yet been implemented.

"lms1xx" is the driver for the LMS111 lidar scanner. LMS1xx.launch should be modified to ensure the proper host address.

"Microstrain_3dmgx2_imu" is the driver for the eponymous IMU. The proper permissions should be set for the port (dev/TTYacm0 for our system).

"robot_localization" provides global pose estimates from sensor data. "lms_sc_ukf_template.launch" was our working file, and should serve as a good starting point, along with http://docs.ros.org/lunar/api/robot_localization/html/index.html

"slam_gmapping" fuses sensor data to perform mapping. *Requires odometry to work: until odometry is implemented successfully, the hector_mapping package is a serviceable substitute, and is currently called from dunn/launch/dunn_transforms.launch*

"us_digital_encoders" is the driver package for the motor encoders. Created by a previous graduate student and modified this year, *does not currently work and we're not sure why*. 

