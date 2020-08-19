# Autonomous Racing Software Stack and Simulation Enviroment

[![Build Status](https://travis-ci.org/arpg-sophisticated/ar-tu-do.svg?branch=development)](https://travis-ci.org/arpg-sophisticated/ar-tu-do)

This repository contains software for 1/10th scale autonomous race cars to compete in the [F1/10 competition](http://f1tenth.org/). It is developed by the Autonomous Racing Project Group of [TU Dortmund](https://ls12-www.cs.tu-dortmund.de/daes/).

## Documentation

* For general information and documentation check out our [wiki page](https://github.com/arpg-sophisticated/ar-tu-do/wiki).
* For source code documentation check out the auto-generated [Doxygen documentation](https://arpg-sophisticated.github.io/doc/index.html).

## Video of final results and testing

<img src="doc/ARPG.gif" alt="Final presentation video" width="850"/>

## Simulation

<img src="doc/racing_example.gif" alt="Racing with a wallfollowing algorithm" width="850"/>

## Features

We provide several LIDAR based driving algorithms:

- Fast and efficient wallfollowing based on fitting circles into the LIDAR scan
- Sensorfusion of ZED camera and LIDAR data
- Boxing of sensor data
- Voxel based obstacle detection (experimental)
- Heavy workload code is in C++
- Full telemetry logging and HUD display
- Report creation of telemetry data
- [ROS navigation stack](http://wiki.ros.org/navigation) based implementation that uses SLAM, a precalculated map and path planning
- Deep Reinforcement Learning ([Q-Learning](https://en.wikipedia.org/wiki/Q-learning) and [Policy Gradient](https://en.wikipedia.org/wiki/Reinforcement_learning#Direct_policy_search))
- Neural Networks with evolutionary training
- Depth camera support
- Video recording
- Huge set of display options in RViz
- Management script

Our software works on physical hardware and in a simulated environment using [Gazebo](http://gazebosim.org/).
Further features are:

- Automatic emergency braking
- Dead Man's Switch
- Teleoperation via keyboard, Xbox and Playstation controller
- Speedometer and Lap Timer

We also added some more stuff not directly connected to the software, please check out the wiki for more information.

## Hardware

Our car is based on a 1/10th scale RC car ([Traxxas Ford Fiesta](https://traxxas.com/products/models/electric/ford-fiesta-st-rally)) with these additions:

- CPU/GPU board ([NVIDIA Jetson](https://www.nvidia.com/object/jetson-tk1-embedded-dev-kit.html))
- motor controller ([FOCBOX](https://www.enertionboards.com/FOCBOX-foc-motor-speed-controller.html))
- LIDAR scanner ([Hokuyo UST-10LX](https://www.hokuyo-usa.com/products/scanning-laser-rangefinders/ust-10lx))
- an inertial measurement unit ([Invensense MPU-9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/))
- Brushless DC motor (replaces the standard brushed motor)
- Stereo camera ([ZED](https://www.stereolabs.com/zed/))

## License

This project (excluding git submodules) is under MIT and GPLv3 dual licensed - see the [MIT.LICENSE](MIT.LICENSE) and [GPLv3.LICENSE](GPLv3.LICENSE) file for details.

<img src="doc/banner.png" alt="Racing with a wallfollowing algorithm" width="850"/>
