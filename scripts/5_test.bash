#!/usr/bin/env bash

docker run -it --rm \
    --name door_adapter_megazo_test_c \
door_adapter_megazo:icadr bash -c \
"source /ros_entrypoint.sh && \
colcon test --packages-select door_adapter_megazo --event-handlers console_cohesion+"