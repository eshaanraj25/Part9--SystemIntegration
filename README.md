# Project: System Integration

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This is the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. In this project we are required to program a Self-Driving Car (Simulation) to work around a lap consisting of Traffic Lights.

Getting Started
---
The project has been developed on a Linux machine with Python2 and ROS Kinetic. The system was provided by Udacity for this particular project.


## Dependencies

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).
* You can check the Udacity Similator repository [here](https://github.com/udacity/self-driving-car-sim)

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Simulator
You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Using the application
---

## Build
```bash
cd ros
catkin_make
```

## Run
After building the project, launch the launch file

```bash
source devel/setup.sh
find /home/workspace/your/directory -type f -iname "*.py" -exec chmod +x {} \;
roslaunch launch/styx.launch
```

Wait for Tensorflow to initialize and then start the simulator.

Results
---
The Traffic Light Detection Model used is described in [REFLECTION.md](REFLECTION.md).

[Youtube video](https://youtu.be/HYewiiKAOAI)
