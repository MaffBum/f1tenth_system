As this PC is running Ubuntu 22 we need to use a container to run ROS2 Foxy. On the host there is a folder called `f1tenth_ws`. In this there is a `src` folder. Open this in VSCode and it will prompt you to open folder in container as there is a .devcontainer folder in src.

This will open a workspace, `ws`, which is a ROS2 Foxy workspace. It has the `f1tenth_system` repo as a folder in src as well as mounting the `f1tenth_gym_ros` in order to run the sim locally (note this is different to autodrive).

WHen you open the devcontainer it already builds all the packages and sources setups.

You can also launch the joy node directly from within this container.

[See this for more details for how to setup the container](https://docs.ros.org/en/foxy/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)

Note I have edited the Dockerfile and devcontainer.json file to suit our needs.


