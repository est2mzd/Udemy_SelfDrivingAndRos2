#!/bin/bash

# common.shを読み込む
source "$(dirname "$0")/common.sh"

#
docker container stop $CONTAINER_NAME

sleep 0.1
docker ps -a