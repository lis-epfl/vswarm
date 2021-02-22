# Installation

The easiest way to get started is to install the [PX4 dev environment](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html), specifically the [`ubuntu_sim_ros_melodic.sh`](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh) script.
It installs many of the required dependencies, including ROS, Gazebo, and [MAVROS](http://wiki.ros.org/mavros):

```bash
wget -q -O - https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh | bash
```

Watch the output of the script closely as you may have to run it twice to make sure everything installs/builds correctly.

## Install Gazebo with physics and sensor lockstep

The default ROS distribution comes with Gazebo 9.0.0 installed.
However, to use [physics and sensor lockstep](http://gazebosim.org/tutorials?tut=lockstep_physics_sensors&cat=sensors) we need at least version 9.16.0 which can be installed by using the [official Ubuntu packages](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0):

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo9
sudo apt upgrade  # this is needed to update libignition!
gazebo --version  # should output version 9.16.0 (or higher)
```

## Install the PX4 autopilot

To install and build PX4, issue the following commands:

```bash
git clone -b v1.10.2 https://github.com/PX4/PX4-Autopilot.git px4
cd px4
echo "export PX4_ROOT=$(realpath .)" >> ~/.bashrc  # or .zshrc
sudo apt install libgstreamer-plugins-base1.0-dev # libgstreamer is a dependency!
sed -i 's/TRUE/true/' Tools/sitl_gazebo/include/gazebo_opticalflow_plugin.h # use boolean instead of macro
make px4_sitl gazebo  # Gazebo should open with a 3DR Iris quadcopter
```

## Install selected ROS packages with Python 3 support

Since we are using Python 3 (specifically version 3.6.9), we need to build the following packages with Python 3 support:

- [`vision_opencv`](https://wiki.ros.org/vision_opencv)
- [`geometry2`](https://wiki.ros.org/geometry2)
- [`orocos_kinematics_dynamics`](https://wiki.ros.org/orocos_kinematics_dynamics)
- [`gazebo_ros_pkgs`](http://wiki.ros.org/gazebo_ros_pkgs)

The `vision_opencv` package is required for converting images using [`cv_bridge`](https://wiki.ros.org/cv_bridge), whereas `geometry2` and `orocos_kinematics_dynamics` are required for frame transformations with [`tf2`](https://wiki.ros.org/tf2/).

To clone the repositories into the workspace, we use `wstool`:

<!-- TODO(fabian): check if this runs! -->

```bash
cd ~/catkin_ws/src
git clone https://github.com/fabianschilling/vswarm.git
wstool init  # should already be initialized!
wstool merge vswarm/vswarm.rosinstall
wstool update
```

Update dependencies using `rosdep`:

```bash
rosdep install --from-paths ~/catkin_ws/ --ignore-src -r -y
```
Tell ROS we'll be using Python 3:

```bash
echo "export ROS_PYTHON_VERSION=3" >> "$HOME"/.bashrc
sed -i 's/python/python3/' ~/catkin_ws/src/gazebo_ros_pkgs/gazebo_ros/scripts/spawn_model
```

Update Python dependencies:

```bash
cd ~/catkin_ws/src/
sudo apt install python3-pip  # required for pip installations
sudo apt install python3-sip-dev  # PyKDL requirement
python3 -m pip install --user -r vswarm/requirements.txt
# This is a bad hack, but it works :) (build requirement for python_orocos_kdl)
ln -sf /usr/lib/python2.7/dist-packages/sipconfig{,_nd}.py ~/.local/lib/python3.6/site-packages/
ln -sf ${PX4_ROOT}/ROMFS ~/.ros/  # Let PX4 find its config (even when working dir is ~/.ros)
```

Before building the above packages, invoke `catkin` with a configuration that contains the following additional CMake arguments:

```bash
cd ~/catkin_ws
catkin config --append-args \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source ~/.bashrc  # Make sure ROS_PYTHON_VERSION=3 is set before you invoke catkin build
catkin build  # You can (probably) safely ignore the warnings
```

Finally, automatically source the `vswarm`-specific environment and define some convenience aliases:

```bash
# The following line sets an empty PYTHONPATH at the beginning of the ~/.bashrc
# This stops nasty shufflying of order in the PYTHONPATH
# We always want our local catkin workspace to overshadow the global one
sed -i '1s/^/export PYTONPATH=\n/' ~/.bashrc
echo "source ~/catkin_ws/src/vswarm/util/setup.zsh" >> ~/.bashrc
echo "alias vswarm='rosrun vswarm swarm_control.py'" >> ~/.bashrc
echo "alias rqt_reconfigure='rosrun rqt_reconfigure rqt_reconfigure'" >> ~/.bashrc
```

### Download the detection model

The `vswarm` drone detection model (~30MB) can be downloaded here: [[Google Drive](https://drive.google.com/file/d/1noeXsnElS1Uyiz72UHIjDU5RXF2G_bIC)] [[SWITCHdrive](https://drive.switch.ch/index.php/s/xa2jtt5sBYruDXK)].

The model is a single-class version of [YOLOv3-tiny](https://arxiv.org/abs/1804.02767) adapted from the [ultralytics/yolov3](https://github.com/ultralytics/yolov3) repository.
The `.cfg` file follows the [Darknet config format](https://github.com/pjreddie/darknet) and the `.pt` file is a serialized Python dictionary with a single `model` key that contains a PyTorch [`state_dict`](https://pytorch.org/tutorials/beginner/saving_loading_models.html).

To make the ROS detection node aware of the detection model you downloaded, set the following environment variable:

```bash
echo "export DETECTION_CHECKPOINT=/path/to/yolov3_tiny.pt" >> ~/.bashrc
```

### Useful tools

The following are not strictly required but are either nice to have or in some cases needed to enable some of the functionality of this repository:

- Shell: [`zsh`](https://www.zsh.org) (with [`oh-my-zsh`](https://ohmyz.sh) ) as a more powerful and user-friendly alternative to [`bash`](https://www.gnu.org/software/bash/)
- Terminal multiplexer: [`tmux`](https://github.com/tmux/tmux) (with the [`tmuxp`](https://github.com/tmux-python/tmuxp) session manager).
