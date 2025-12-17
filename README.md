[![build](https://github.com/MEGAZO-io/door_adapter_megazo/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/MEGAZO-io/door_adapter_megazo/actions/workflows/industrial_ci_action.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

# What Is This?
This repository contains the Robotics Middleware Framework (**RMF**) **Door Adapter** for interfacing with **MEGAZO Technologies**'s ICADR-based IoT-enabled doors.

| :point_up: Looking to convert a legacy door to allow robot-human door sharing?               |
|:----------------------------|
| **MEGAZO ICADR** is an IoT (Internet of Things) device that takes in dry contacts from the door and makes it cloud-accessible! |

It is fully compatible with [RMF](https://github.com/open-rmf/rmf_ros2) and [SS-713](https://www.singaporestandardseshop.sg/Product/SSPdtDetail/4bc72ff7-951f-4a5e-b85f-edafe36a8a3d).

> [!NOTE]  
> Contact [hello@megazo.io](hello@megazo.io) for more details.


# **Dependencies** 📚
- ROS 2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)

# **Configurations**
Contact [hello@megazo.io](hello@megazo.io) to be provided with API credentials which will be inserted in [config.yaml](door_adapter_megazo/config.yaml).

# **Build** 🔨

1. **Download** `door_adapter_megazo` repository:

```bash
git clone https://github.com/MEGAZO-io/door_adapter_megazo.git --depth 1 --single-branch && cd door_adapter_megazo
```

2. **Build** docker image:

```bash
docker build -t door_adapter_megazo:icadr .
```

# **Run** ⚙️

```bash
docker run -it --rm \
	--name door_adapter_megazo_c \
	--network host \
	-e RCUTILS_COLORIZED_OUTPUT=1 \
door_adapter_megazo:icadr bash -c "source /ros_entrypoint.sh && ros2 launch door_adapter_megazo run.launch.xml"
```

# **Verify** ✅

Upon running the command above, you should see a terminal output similar to what is shown below:

```bash
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-12-17-14-18-12-752985-bastion-1
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [door_adapter-1]: process started with pid [33]
[door_adapter-1] [INFO] [1765981093.171074943] [door_adapter_megazo]: Initialising [ door_adapter_megazo ]...
[door_adapter-1] [INFO] [1765981093.490804805] [door_adapter_megazo]: [ door_adapter_megazo ] - [ RUNNING ]...
[door_adapter-1] [WARN] [1765981093.508577969] [door_adapter_megazo]: Connection to MQTT Broker - [ SUCCESS ]
```