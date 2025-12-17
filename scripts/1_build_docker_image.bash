#!/usr/bin/env bash

docker build --progress=plain -t door_adapter_megazo:icadr .
docker image prune -f
