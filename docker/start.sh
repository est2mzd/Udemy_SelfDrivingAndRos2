#!/bin/bash

# common.shを読み込む
source "$(dirname "$0")/common.sh"

# Dockerコンテナをデタッチモードで起動
# コンテナ内とディスプレイを共有するために下記を追加
#    --env "DISPLAY=$DISPLAY" \
#    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
docker run \
    --rm \
    -itd \
    -v "$WORK_DIR":/work \
    --name "$CONTAINER_NAME" \
    --workdir /work \
    --user $USER_NAME:$USER_NAME \
    --env  USERNAME=$USER_NAME \
    --env  USERID=$USER_ID \
    --env "DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "$IMAGE_NAME"