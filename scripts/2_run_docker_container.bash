#!/usr/bin/env bash

CONFIG_PATH="/home/megazo_door_ws/config.yaml"

echo -n "Enable DEBUG logging? (y/N): "
read answer

# Default to INFO if empty or not y/Y
if [[ "$answer" == "y" || "$answer" == "Y" ]]; then
    LOG_LEVEL="DEBUG"
else
    LOG_LEVEL="INFO"
fi

echo "Selected log level: $LOG_LEVEL"

docker run -it --rm \
	--name door_adapter_megazo_c \
	--network host \
	-e RCUTILS_COLORIZED_OUTPUT=1 \
door_adapter_megazo:icadr bash -c "source /ros_entrypoint.sh && \
ros2 launch door_adapter_megazo run.launch.xml \
log_level:=$LOG_LEVEL"
