# vswarm

The `vswarm` package enables decentralized vision-based control of drone swarms without relying on inter-agent communication or visual fiducial markers.

This repository contains the source code accompanying our article:

F. Schilling, F. Schiano, and D. Floreano, "**Vision-based Drone Flocking in Outdoor Environments**," *IEEE Robotics and Automation Letters (RA-L)*, vol. 6, no. 2, pp. 2954-2961, 2021. [[arXiv](https://arxiv.org/abs/2012.01245)] [[IEEE *Xplore*](https://ieeexplore.ieee.org/document/9363551)] [[Citation](#citation)]

The package provides its main functionality in the following modules:

- Visual **detection** of neighboring drones from omnidirectional images using a convolutional neural network
- Relative **localization** based on camera parameters and the known physical size of the drones
- Multi-target **tracking** to estimate the position and velocity of the drones
- Multi-agent **control** using a Reynolds-inspired flocking algorithm

The following [video](https://youtu.be/wU8-Wm9_YLs) provides a high-level overview of the method and a demonstration of `vswarm` in action:

[![vswarm](./docs/vswarm.png)](https://youtu.be/wU8-Wm9_YLs)

## Requirements

This package requires [Ubuntu 18.04 (Bionic)](https://releases.ubuntu.com/18.04/) and [ROS Melodic](https://wiki.ros.org/melodic).
The simulation environment is based on [Gazebo](http://gazebosim.org).
The majority of the code is written in Python 3.

## Installation

The installation instructions for our simulation environment can be found in a dedicated [`installation.md`](./docs/installation.md).
The code can also run onboard a physical drone with a suitable companion computer such as the [NVIDIA Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2).

## Usage

To launch three drones in Gazebo:

```bash
roslaunch vswarm vswarm_gazebo.launch n:=3
```

Use the `vswarm` command to let the drones take off and switch to offboard mode (i.e. to start the vision-based flocking algorithm):

```bash
vswarm takeoff    # Drones take off and hover at 2.5m altitude
vswarm offboard   # Drones start flocking
```

By default, the drones are migrating in a square according to a set of predefined waypoints.

The parameters of the most important nodes (object detection, relative localization, multi-target tracking, and flocking) can be adjusted using the dynamic reconfigure GUI:

```
rqt_reconfigure
```

*Note:* The reason for the relatively low confidence score of the detections despite the visually simple environment is the fact that the detection model has never been trained on simulated data (but on the [dataset](#dataset) below).

## Dataset

The `vswarm` drone detection dataset (~2GB) can be downloaded here: [[Google Drive](https://drive.google.com/file/d/1aU1ZLS8TsWAC9mf5PRZBgN2o6kItWQvL)] [[SWITCHdrive](https://drive.switch.ch/index.php/s/6LT9qQEZQg5HyYg)].

For more information about the dataset, check out the [`dataset.md`](./docs/dataset.md)

## Citation

If you use this work in an academic context, please cite the following article:

```bibtex
@article{schilling_vswarm_2021,
    title   = {Vision-based Drone Flocking in Outdoor Environments},
    author  = {Schilling, Fabian and Schiano, Fabrizio and Floreano, Dario},
    journal = {IEEE Robotics and Automation Letters},
    year    = {2021},
    volume  = {6},
    number  = {2},
    pages   = {2954--2961},
    doi     = {10.1109/LRA.2021.3062298},
    issn    = {2377-3766},
    pages   = {8}
}
```

## Acknowledgments

We used a variety of third party tools and software libraries which we would like to acknowledge:

- [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) for camera calibration
- [ultralytics/yolov3](https://github.com/ultralytics/yolov3) for training the detector
- [facontidavide/plotjuggler](https://github.com/facontidavide/PlotJuggler) for visualizing [PX4 ulog](https://docs.px4.io/master/en/dev_log/ulog_file_format.html) and [ROS bag](https://wiki.ros.org/rosbag) files
- [rfs_tracking_toolbox](http://ba-tuong.vo-au.com/codes.html) for heavy inspiration of the filter implementation

## License

This project is released under the [MIT License](https://opensource.org/licenses/MIT).
Please refer to the [`LICENSE`](LICENSE) for more details.
