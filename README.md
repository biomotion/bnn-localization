# bnn-localization

This is a repo for localization competition in self-driving car course

## Software Requirements

- Docker ce ([Installation guide](https://docs.docker.com/engine/install/ubuntu/))

## Branch

We have the following branch for itri and nuScenes dataset respectively:

- `devel-itri` for running itri dataset
- `devel-nuscenes` for running itri dataset

try `git checkout <branch name>` to switch to the correct branch.

## How to Run

First, you should enter the container.

```bash
source docker_run.sh
```

For the first time use, or whenever you have some new code in the workspace:

```bash
catkin_make
```

For running the main icp function:

Take a look at `catkin_ws/src/icp_localization/launch/serial.launch`

After you change all the path and attribute correctly, run the command above

```bash
roslaunch icp_localization serial.launch
```
