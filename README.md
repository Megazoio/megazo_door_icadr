# What is this?
This repository contains the RMF door adapter for Megazo ICADR.  

Looking to convert a legacy door to allow robot-human door sharing? Megazo ICADR is an IoT device that takes in dry contacts from the door and convert it to a cloud accessible door. It is fully compatible with [RMF](https://github.com/open-rmf/rmf_ros2) and [SS713](https://www.singaporestandardseshop.sg/Product/SSPdtDetail/4bc72ff7-951f-4a5e-b85f-edafe36a8a3d).  
Contact hello@megazo.io for more details.

# Dependencies
ROS2 Humble

# Configurations
Contact hello@megazo.io for log in credentials to be inserted in [config.yaml](megazo_door_icadr/config.yaml)

# Build
source /opt/ros/humble/setup.bash  
colcon build

# Run
source ~/xx_ws/install/setup.bash  
ros2 launch megazo_door_icadr run.launch.xml

