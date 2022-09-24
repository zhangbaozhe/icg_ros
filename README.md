# ICG ROS Wrapper

This is a ROS wrapper of the implementation of a pose tracking
library ICG developed by DRL. The original repo can be found [here](https://github.com/DLR-RM/3DObjectTracking/tree/master/ICG).

If you are interested in this work and want to use it in your project, please site it with
```text
@InProceedings{Stoiber_2022_CVPR,
    author    = {Stoiber, Manuel and Sundermeyer, Martin and Triebel, Rudolph},
    title     = {Iterative Corresponding Geometry: Fusing Region and Depth for Highly Efficient 3D Tracking of Textureless Objects},
    booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
    month     = {June},
    year      = {2022},
    pages     = {6855-6865}
}
```

## Environment Settings

This wrapper is tested on a computer with
- ROS noetic
- Ubuntu 20.04

The wrapper is compiled in C++ 17.

## Installation

Before proceeding with this wrapper, please check the README file of the ICG repo to install dependencies [link](https://github.com/DLR-RM/3DObjectTracking/blob/master/ICG/readme.md#build).

This wrapper uses an additional package `gflags` for parsing the command-line arguments.
You can install it using 
```bash
sudo apt-get install libflags-dev
```

After installing all the dependencies, you should compile this package as 
a normal ROS package using `catkin_make`.

A sample node `icg_test_node` is provided. To run this node, you need to first
launch a camera node and make sure there are valid color and depth images. In this node, 
the topics of the realsense camera are used. You should also feed it with
the corresponding configuration directory in the command-line arguments. For example, if you 
have already launched the camera node
```bash
cd <your-ros-workspace>
source devel/setup.bash
rosrun icg_ros icg_test_node -config_dir=src/icg_ros/config
```
The configuration directory in this example contains a triangle 3D model, 
a yaml description of this model, and a description of the detector.

For more information of these configuration files, please refer to the original
repo.