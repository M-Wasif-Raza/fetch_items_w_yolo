xhost local:root
XAUTH=/tmp/.docker.xauth

docker run -it --user wasif \
    --name=ros_noetic_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.x11-unix:/tmp/.x11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH=$XAUTH" \
    --volume=/home/wasif/Documents/Fiverr_Order_4_YOLO_Cafe/docker_folder/main_folder:/home/wasif/main_folder \
    --net=host \
    --privileged \
    ros_noetic_img \
    bash

echo "Done."