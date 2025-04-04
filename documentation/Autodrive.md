To use the Autodrive Simulator you need to start two different containers:
- AutoDrive Simulator
- Autodrive devkit

Ensure that the two images have already been built. You can do this by running this inside of AutoDRIVE-F1TENTH-Sim-Racing (this folder contains the dockerfiles). Note this example is for building the devkit. (NOTE this has already been done)
```bash
docker build --tag autodriveecosystem/autodrive_f1tenth_api:<TAG> -f autodrive_devkit.Dockerfile .
```

**To start the Autodrive Sim:** \
Enable display forwarding
```bash
xhost local:root
```
Run the container at entrypoint
```bash
docker run --name autodrive_f1tenth_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --env ROS_DOMAIN_ID=42 --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:local-latest
```
Once inside the container launch the sim:
```bash
./AutoDRIVE\ Simulator.x86_64 
```

**To start the devkit**

Enable display forwarding
```bash
xhost local:root
```

Run the container at entrypoint
```bash
docker run --name autodrive_f1tenth_devkit   --rm -it   --network=host   --ipc=host   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   --env ROS_LOCALHOST_ONLY=0   --env DISPLAY   --privileged   --gpus all   -v /home/f1tenth/f1tenth_ws/src/f1tenth_system:/home/autodrive_devkit/src/f1tenth_system   autodrive_f1tenth_devkit:local-latest
```

Rviz should launch automatically. But to bringup rviz you can run:
```bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```

Start additional bash session(s) within the container (each in a new terminal window):
```bash
docker exec -it autodrive_f1tenth_devkit bash
```

Note this has been added in the dockerfile to allow communication from the ros2 foxy container running on the PC and the devkit container.
```bash
# Install Cyclone DDS RMW implementation
RUN apt-get update && apt-get install -y \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-ackermann-msgs \
 && rm -rf /var/lib/apt/lists/*

# Set Cyclone DDS as default (optional, but recommended)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=42
```

## Running our agent (other nodes)
Ensure communication is using CycloneDDS. This is mainly for communication between docker containers.
```bash
sudo apt update
sudo apt install ros-foxy-rmw-cyclonedds-cpp
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Autodrive uses `/autodrive/f1tenth_1/throttle_command` and `/autodrive/f1tenth_1/steering_command` instead of the `/drive` topic therefore I have created a `drive_bridge` node which handles this. We can't simply remap as we can't split up `/drive` directly in the launch file.
It also uses `/autodrive/f1tenth_1/lidar` instead of `/scan`. This can be simply remapped either in a launch file or from the command line when running a node, like so
```bash
ros2 run gap_follow test_node.py --ros-args --remap /scan:=/autodrive/f1tenth_1/lidar
```

This way we avoid directly editing the existing `autodrive_bridge` and our code will still publish to the `drive` topic. This means it should still work on the gym and the real car.

You can check communication by running the talker/listener nodes or checking rqt_graph once launching a node.

[See this for more details](https://autodrive-ecosystem.github.io/competitions/f1tenth-sim-racing-guide/#21-system-requirements)

[For comp rules see here](https://autodrive-ecosystem.github.io/competitions/f1tenth-sim-racing-rules/#3-submission-guidelines)