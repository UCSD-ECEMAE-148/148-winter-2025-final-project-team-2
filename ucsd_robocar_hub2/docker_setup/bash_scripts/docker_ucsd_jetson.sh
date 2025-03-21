docker run \
    --name ${1:-test} \
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --device /dev/video0 \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    djnighti/ucsd_robocar:${2:-latest}
