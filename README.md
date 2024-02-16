# ros2_coco_detector
Integrate PyTorch MobileNet for Microsoft COCO object detection into ROS2 environment

## Packages

coco_detector: package containing coco_detector_node for listening on ROS2 topic /image and publishing ROS2 Detection2DArray message on topic /detected_objects. Also (by default) publishes Image (with labels and bounding boxes) message on topic /annotated_image. The object detection is performed by PyTorch using MobileNet.

### Tested Hardware


### Tested Software


## Installation

Follow the [RoboStack](https://robostack.github.io/GettingStarted.html) installation instructions to install ROS2

(Ensure you have also followed the step Installation tools for local development in the above instructions)

Follow the [PyTorch](https://pytorch.org/) installation instructions to install PyTorch (selecting the conda option).

```
mamba activate ros2  # (use the name here you decided to call this conda environment)
mamba install ros-humble-image-tools
cd ~
mkdir -p ros2_ws/src
cd ros2_ws
git -C src clone https://github.com/jfrancis71/ros2_coco_detector.git
colcon build --symlink-install
```
You may receive a warning on the colcon build step: "SetuptoolsDeprecationWarning: setup.py install is deprecated", this can be ignored.

The above steps assume a RoboStack mamba/conda ROS2 install. If using other installation process, replace the RoboStack image-tools package install step with whichever command is appropriate for your environment. The image-tools package is not required for coco_detector, it is just used in the steps below for convenient demonstration.

## Activate Environment

```
mamba activate ros2 # (use the name here you decided to call this conda environment)
cd ~/ros2_ws
source ./install/setup.bash
```

## Verify Install

Launch a camera stream:
```
ros2 run image_tools cam2image
```

On another terminal enter:
```
ros2 run coco_detector coco_detector_node
```
There will be a short delay the first time the node is run for PyTorch TorchVision to download the neural network. You should see a downloading progress bar. This network is then cached for subsequent runs.

On another terminal to view the detection messages:
```
ros2 topic echo /detected_objects
```
To view the image stream annotated with the labels and bounding boxes:
```
ros2 run image_tools showimage --ros-args -r /image:=/annotated_image
```

## Suggested Setup For Mobile Robotics


## External Links

[COCO Home](https://cocodataset.org/#home)

[Microsoft COCO: Common Objects in Context](http://arxiv.org/abs/1405.0312)

[PyTorch MobileNet](https://pytorch.org/vision/stable/models/generated/torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn.html)

[MobileNets: Efficient Convolutional Neural Networks for Mobile Vision Applications](https://arxiv.org/abs/1704.04861)
