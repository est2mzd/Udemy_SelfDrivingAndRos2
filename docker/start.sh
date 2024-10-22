#!/bin/bash

# common.shを読み込む
source "$(dirname "$0")/common.sh"

# Dockerコンテナをデタッチモードで起動
docker run \
    --rm \
    -itd \
    -v "$WORK_DIR":/work \
    --name "$CONTAINER_NAME" \
    --workdir /work \
    --user $USER_NAME:$USER_NAME \
    --env  USERNAME=$USER_NAME \
    --env  USERID=$USER_ID \
    "$IMAGE_NAME"