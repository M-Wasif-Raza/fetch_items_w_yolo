xhost +local:root
XAUTH=/tmp/.docker.xauth

docker run -it --user wasif \
    --name=ros_noetic_container_3 \
    --memory=8g \
    --shm-size=4g \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.x11-unix:/tmp/.x11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH=$XAUTH" \
    --volume=/home/wasif/Documents/Fiverr/Fiverr_Order_4_YOLO_Cafe/docker_folder/main_folder:/home/wasif/main_folder \
    --net=host \
    --privileged \
    --device /dev/snd \
    --env="PULSE_RUNTIME_PATH=/var/run/pulse" \
    --env="PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native" \
    --volume="${XDG_RUNTIME_DIR}/pulse:${XDG_RUNTIME_DIR}/pulse" \
    --group-add audio \
    fetch_yolo_img \
    bash

echo "Done."