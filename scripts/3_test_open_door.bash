#!/usr/bin/env bash

DOOR_NAME="XXXXXXXXXXXXXX"

docker exec -it door_adapter_megazo_c bash -c "source /ros_entrypoint.sh && ros2 topic pub --once /adapter_door_requests rmf_door_msgs/msg/DoorRequest \"{request_time: {sec: 1633024800, nanosec: 123456789}, door_name: '$DOOR_NAME', requester_id: 'admin', requested_mode: {value: 2}}\""

unset DOOR_NAME