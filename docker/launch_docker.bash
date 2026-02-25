#!/bin/bash
xhost local:root

rm -r /tmp/.dockersim.xauth || true
touch /tmp/.dockersim.xauth
xauth nlist :1 | sed -e 's/^..../ffff/' | xauth -f /tmp/.dockersim.xauth nmerge -

docker run -it \
    --name flatland \
    --rm \
    --net=host \
    --privileged \
    --device=/dev/bus/usb:/dev/bus/usb \
    -e DISPLAY=$DISPLAY \
    -e PYTHONBUFFERED=1 \
    -e ROS_LOCALHOST_ONLY=0 \
    --ipc=host \
    --pid=host \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /run/user/1000/gdm/Xauthority:/root/.Xauthority:ro \
    -v ./../../:/root/flatland_ws/src/ \
    flatland_sim:latest bash
