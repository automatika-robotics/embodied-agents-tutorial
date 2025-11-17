# Tutorial for ROSCon Strasbourg

## Getting Started
1. The easiest way to get started is to install docker and run a jazzy ROS container with the following:

```shell
docker run -it --network host ros:jazzy-ros-base
```

2. Install the packages as per installation instructions sent in the email. If you want to use your laptop camera for multimodal applications install the following package:

`sudo apt install ros-<VERSION>-usb-cam`

We will run it as follows in a terminal:

`ros2 run usb_cam usb_cam_node_exe`

3. As we run recipes, you will be prompted for installing additional packages based on recipe requirements. First recipe requires `pip install ollama`
