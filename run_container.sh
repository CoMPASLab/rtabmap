#!/usr/bin/env bash

xhost +local:root

docker run -it \
    --name rtabmap \
    --privileged \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    --net=host \
    $@ \
    rtabmap

xhost +local:root