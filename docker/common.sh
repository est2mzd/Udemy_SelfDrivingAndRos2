#!/bin/bash

# build.sh に必要な変数
IMAGE_NAME="ros2_"$USER
DOCKERFILE_DIR=$(dirname "$0")

# ホストのユーザー情報
USER_NAME=$(whoami)
USER_ID=$(id -u)
#USER_ID=1001

# start.sh に必要な変数
CONTAINER_NAME=$IMAGE_NAME

# このファイルの1個上をマウントする
#WORK_DIR=$(dirname "$(dirname "$0")")
WORK_DIR=$(dirname "$(dirname "$(realpath "$0")")")