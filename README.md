## Steps to run this project (in order given below)
- ros2 run differential_drive twist_to_motors (for running 2 motors and robot arm)
- ros2 run teleop_twist_keyboard teleop_twist_keyboard (optional, only if you want to control robot using keyboard)

  Both above commands will get current encoder readings and we can give DC motor values and robot arm values.
- ros2 launch studica_rbt_hardware_launch.py (used for getting important static transforms)
- ros2 launch ydlidar_ros2_driver ydlidar_launch.py ()
- ros2 launch orbbec_camera gemini_e.launch.py
- ros2 launch amr_vision stdca_rbt_aprltg_vision_launch.py
- ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/<rgb_image_topic> \
    -r camera_info:=/<rgb_camera_info_topic> \
    --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
- ros2 launch rbt_navigtn rbt_mapping_launch.py (for mapping) OR ros2 launch rbt_navigtn rbt_navigation_launch.py (for navigation)
## Other information
Some things are optional like setting up IMU of MPU 6050 and setting up 2D LIDAR Odometry. 2 ESP32 are used, one which is soldered on purf board is for sending commands to robot arm and motor driver.
Other ESP32 is for getting encoder data of 2 motors. Robot arm, 2 ESP32, 2D Lidar and camera are connected to Jetson Orin Nano. A battery is connected in parallel to the circuit. Two types of gripper can be used,
out of which one is for lifting beakers ,test tubes etc. and the other one is for lifting tablets.
## Problems present currently and further development needed
The soldered ESP32 sometimes start the motors thus causing issues in stopping them using teleoperation. I have to put the USB wiring serially in order so that ttyUSBnumber is taken by the node which I am running.
Gripper should get rid of mechanical errors which might occur. Robot arm should be tested using ROS2 for its movements. Nav2 should be used for giving robot a point where it can then navigate and then april tag placed
there can be detected by it. The robot arm end effector should reach close to the tag and then it can go to its initial position. The robot will then return to its home position by doing navigation autonomously using nav2.
A C++ or python code can be helpful in achieving this.
