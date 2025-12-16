#!/usr/bin/env bash

docker build -t door_adapter_megazo:icadr .
docker image prune -f
