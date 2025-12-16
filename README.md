# What Is This?
This repository contains the Robotics Middleware Framework (RMF) Door Adapter for MEGAZO Technologies's ICADR-based IoT-enabled doors.

Looking to convert a legacy door to allow robot-human door sharing? Megazo ICADR is an IoT device that takes in dry contacts from the door and convert it to a cloud accessible door. 

It is fully compatible with [RMF](https://github.com/open-rmf/rmf_ros2) and [SS-713](https://www.singaporestandardseshop.sg/Product/SSPdtDetail/4bc72ff7-951f-4a5e-b85f-edafe36a8a3d).

Contact [hello@megazo.io](hello@megazo.io) for more details.

# **Dependencies**
- ROS 2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)

# **Configurations**
Contact [hello@megazo.io](hello@megazo.io) for log in credentials to be inserted in [config.yaml](door_adapter_megazo/config.yaml).

# **Build**

1. **Download** `door_adapter_megazo` repository:

```bash
git clone https://github.com/MEGAZO-io/door_adapter_megazo.git --depth 1 --single-branch && cd door_adapter_megazo
```

2. **Build** docker image:

```bash
docker build -t door_adapter_megazo:icadr .
```

# **Run**

```bash
docker run -it --rm \
	--name door_adapter_megazo_c \
	--network host \
door_adapter_megazo:icadr bash -c "source /ros_entrypoint.sh && ros2 launch door_adapter_megazo run.launch.xml"
```

