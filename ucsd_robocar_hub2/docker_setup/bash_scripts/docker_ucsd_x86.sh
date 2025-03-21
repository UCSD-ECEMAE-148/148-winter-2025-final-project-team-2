docker run \
    --name ${1:-test} \
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --device /dev/video0 \
    djnighti/ucsd_robocar:${2:-x86}
