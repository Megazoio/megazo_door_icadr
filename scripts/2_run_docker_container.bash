#!/usr/bin/env bash

docker run -it --rm \
	--name door_adapter_megazo_c \
	--network host \
door_adapter_megazo:icadr bash -c "source /ros_entrypoint.sh && ros2 launch door_adapter_megazo run.launch.xml"
