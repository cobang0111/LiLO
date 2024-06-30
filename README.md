# Real-time Optimization of Link Length for highly efficient quadruped robots (ROLL) 
## ✨Summary
This repository about real-time optimization of input link length for 4bar linkage based quadruped robots.
We suggest new mechanism that adjust input link length using sub actuator and optimal length control method.
<br>
<br>
[Youtube Video1](https://youtu.be/Fd4iINc_CYo?si=tXBfKRfczNwu97U1)
<br>
<br>
[Youtube Video2](https://youtu.be/Z9Yeyp4EUFA?si=rExXM8d1uZO81aSZ)

<br>

![image]()


<br>
Tested on MATLAB Simulink R2024a
Hardware AK80-9 Motor(hip) - AK80-9 Motor(knee) - Bl2446s Motor(Input Link Length) - TI launchpad f28379d(Motor Driver) 

<br>

<img src="" width="480">

<br>
<br>

## ✨Overview
This repository assumes 


<br>

I have an issue when start the docker container.
If you so, Please start docker container using below command.
```bash
docker run --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it --privileged --net=host isaac_ros_dev-x86_64:latest
```

<br>

After you test the example of isaac_ros_pose_estimation centerpose, 
Install Intel RealSense SDK in docker container
```bash
xhost +local:root

docker start <container ID>

docker exec -it <container ID> bash

cd ~

git clone https://github.com/IntelRealSense/librealsense.git

sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake

sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

cd librealsense

sudo ./scripts/setup_udev_rules.sh

./scripts/patch-realsense-ubuntu-lts-hwe.sh

echo 'hid_sensor_custom' | sudo tee -a /etc/modules

mkdir build && cd build

cmake ../ -DBUILD_EXAMPLES=true

make -j$(nproc)

sudo make install

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc

source ~/.bashrc

realsense-viewer
```

<br>

<br>

## ✨Install
Please clone this repository under /workspace/isaac_ros-dev/src
```bash
cd /workspace/isaac_ros-dev/src

git clone https://github.com/cobang0111/centerpose_pick_and_place.git

cd ..
```

And then update bashrc file and rebuild the packages
```bash
nano ~/.bashrc

source /opt/ros/humble/setup.bash
source /workspace/isaac_ros-dev/install/setup.bash

colcon build

source ~/.bashrc
```
You may need to modify the code `src/centerpose_pick_and_place.cpp` appropriately to your case!!!


<br>

## ✨Execution
1st terminal (Activating x server + starting docker + Executing intel realsense node)



```
xhost +local:root

docker start <containter-id>

docker exec -it <containter-id> bash

cd /workspace/isaac_ros-dev

ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=848x480x30 depth_module.profile:=848x480x30 pointcloud.enable:=true rgb_camera.color_format:=bgr8
```


<br>
<br>





