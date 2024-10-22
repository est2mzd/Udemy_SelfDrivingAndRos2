#!/bin/bash

# common.shを読み込む
source "$(dirname "$0")/common.sh"

# Dockerイメージをビルド
docker build \
    --build-arg USERNAME=$USER_NAME \
    --build-arg USERID=$USER_ID \
    -t "$IMAGE_NAME" "$DOCKERFILE_DIR"
