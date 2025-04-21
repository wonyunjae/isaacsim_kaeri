#!/bin/sh

# get input IMAGE_NAME
echo "Enter the image name: "

# echo all images
echo "-- option images : "
docker images
echo "-- "
read IMAGE_NAME

# get input CONTAINER_NAME
echo "Enter the container name: "
read CONTAINER_NAME

# echo image name and container name
echo "=-=-=-=-=-=-=-=-=-="
echo "IMAGE_NAME: $IMAGE_NAME"
echo "CONTAINER_NAME: $CONTAINER_NAME"
echo "=-=-=-=-=-=-=-=-=-="

# get input ISAAC_SIM_PATH
echo "Enter the isaac sim path: "
read ISAAC_SIM_PATH

# # get input ORBIT_PATH
# echo "Enter the orbit path: "
# read ORBIT_PATH

xhost +
docker run -dit --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
  -e "PRIVACY_CONSENT=Y" \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -e DISPLAY \
  -v $ISAAC_SIM_PATH:/isaac-sim/isaac_sim \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v $ISAAC_SIM_PATH/kaeri_examples:/isaac-sim/isaac_sim/kaeri_examples \
  --name "$CONTAINER_NAME" \
  "$IMAGE_NAME" \
