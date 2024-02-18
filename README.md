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
mamba install ros-humble-vision-msgs
cd ~
mkdir -p ros2_ws/src
cd ros2_ws
git -C src clone https://github.com/jfrancis71/ros2_coco_detector.git
colcon build --symlink-install
```
You may receive a warning on the colcon build step: "SetuptoolsDeprecationWarning: setup.py install is deprecated", this can be ignored.

The above steps assume a RoboStack mamba/conda ROS2 install. If using other installation process, replace the RoboStack image-tools and vision-msgs packages install steps with whichever command is appropriate for your environment. The image-tools package is not required for coco_detector, it is just used in the steps below for convenient demonstration. However vision-msgs is required (this is where the ROS2 DetectionArray2D message is defined)

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

These suggestions are for a Raspberry Pi 3 Model B+ running ROS2.

As of 16/02/2024, the PyTorch Conda install does not appear to be working for Raspberry Pi 3 Model B+.
There may be other installation options, but I have not explored that.

As an alternative if you have a ROS2 workstation connected to the same network, I suggest publishing the compressed image on the Raspberry Pi and running the COCO detector on the workstation.

The below setup involves the ROS2 compression transport on both the Raspberry Pi and workstation. If using RoboStack ROS2 Humble you can install on each with:

```mamba install ros-humble-compressed-image-transport```

Raspberry Pi (run each command in seperate terminals):

```ros2 run image_tools cam2image --ros-args -r /image:=/charlie/image```

```ros2 run image_transport republish raw compressed --ros-args -r in:=/charlie/image -r out/compressed:=/charlie/compressed```

Workstation (run each command in seperate terminals):

```ros2 run image_transport republish compressed raw --ros-args -r /in/compressed:=/charlie/compressed -r /out:=/server/image```

```ros2 run coco_detector coco_detector_node --ros-args -r /image:=/server/image```

I have relabelled topic names for clarity and keeping the image topics on the different machines seperate. Compression is not necessary, but I have poor performance on my network without compression.

Note you could use launch files (for convenience) to run the above nodes. I do not cover that here as it will be specific to your setup.

## External Links

[COCO Home](https://cocodataset.org/#home)

[Microsoft COCO: Common Objects in Context](http://arxiv.org/abs/1405.0312)

[PyTorch MobileNet](https://pytorch.org/vision/stable/models/generated/torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn.html)

[MobileNets: Efficient Convolutional Neural Networks for Mobile Vision Applications](https://arxiv.org/abs/1704.04861)
