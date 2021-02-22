#!/usr/bin/env zsh

if [[ -z "${PX4_ROOT}" ]]; then
    echo "PX4_ROOT must be set!"
    return 1
fi

pushd "${PX4_ROOT}" > /dev/null

PX4_BUILD_ROOT=$(pwd)/build/px4_sitl_default

if [ ! -d "${PX4_BUILD_ROOT}" ]; then
    pushd "${PX4_BUILD_ROOT}" > /dev/null
    make px4_sitl gazebo
    popd > /dev/null
fi

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default &> /dev/null
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(rospack find vswarm)/models

popd > /dev/null
