# Generative Grasping CNN (GG-CNN) Grasping with Kinova Mico

This repository contains a ROS package for running the GG-CNN grasping pipeline on a Kinova Mico arm.  For the GG-CNN implementation and training, please see [https://github.com/dougsm/ggcnn](https://github.com/dougsm/ggcnn).

The GG-CNN is a lightweight, fully-convolutional network which predicts the quality and pose of antipodal grasps at every pixel in an input depth image.  The lightweight and single-pass generative nature of GG-CNN allows for fast execution and closed-loop control, enabling accurate grasping in dynamic environments where objects are moved during the grasp attempt.

## Paper

**Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach**

*[Douglas Morrison](http://dougsm.com), [Peter Corke](http://petercorke.com), [Jürgen Leitner](http://juxi.net)*

Robotics: Science and Systems (RSS) 2018

[arXiv](https://arxiv.org/abs/1804.05172) | [Video](https://www.youtube.com/watch?v=7nOoxuGEcxA)

If you use this work, please cite:

```text
@article{morrison2018closing, 
	title={Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach}, 
	author={Morrison, Douglas and Corke, Peter and Leitner, Jürgen}, 
	booktitle={Robotics: Science and Systems (RSS)}, 
	year={2018} 
}
```


## Installation

This code was developed with Python 2.7 on Ubuntu 16.04 with ROS Kinetic.  Python requirements can be found in `requirements.txt`.

You will also require the [Kinova ROS Packages](https://github.com/Kinovarobotics/kinova-ros) and [Realsense Camera Packages](http://wiki.ros.org/realsense_camera).

A 3D printed mount for the Intel Realsense SR300 on the Kinova Mico arm can be found in the `cad` folder.

## GG-CNN Model

See [https://github.com/dougsm/ggcnn](https://github.com/dougsm/ggcnn) for instructions for downloading or training the GG-CNN model.

## Running

This implementation is specific to a Kinova Mico robot and Intel Realsense SR300 camera.

Once the ROS package is compiled and sourced:

1. Lanuch the robot `roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=m1n6s200`
2. Start the camera `roslaunch ggcnn_kinova_grasping wrist_camera.launch`
3. Run the GG-CNN node `rosrun ggcnn_kinova_grasping run_ggcnn.py`
4. To perform open-loop grasping, run `rosrun ggcnn_kinova_grasping kinova_open_loop_grasp.py`, or to perform closed-loop grasping run `rosrun kinova_closed_loop_grasp.py`.

**Contact**

Any questions or comments contact [Doug Morrison](mailto:doug.morrison@roboticvision.org).
