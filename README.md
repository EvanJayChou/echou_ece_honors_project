# <div align="center">Uncertainty-Guided Autonomous Exploration via Online 3D Gaussian Splatting</div>
### <div align="center"> Undergraduate ECE Honors Program Project </div>
#### <div align="center"> Evan Chou (ECE: Machine Learning and Controls), FA26 - WI27 </div>
![image](https://raw.githubusercontent.com/UCSD-ECEMAE-148/148sp26-spring-2026-final-project-team-4/main/docs/media/car.jpg)

## Abstract

Our project aimed to develop a SLAM and autonomous navigation system capable of fusing multiple sensors for mapping a full indoor room and localizing the robot's position. With telemetry data and logging from the ROS 2 system, we also aimed to expose the robot through a web-based controller and dashboard for remote control, and to generate mission reports from its exploration data.

<hr>

## What We Promised
### Must Have
* Integrate SLAM and autonomous navigation into the ROS2 system through pre-built packages
* Conduct mission exploration and publish reports of findings into a full-stack website
* Rotational camera for scanning and examining surrounding environment at wider angles

### Nice to Have
* Computer vision and object detection/avoidance with standard YOLO model
* LLM-based summarization of reported findings through on-board edge inference
* Cosmetic additions such as LED strip for indicating exploration status 

<hr>

## Accomplishments

- Successfully launched visual SLAM with an Extended Kalman Filter for robot localization
- Integrated a web controller and dashboard for robot control via ROS 2 socket bridges
- Configured IMU odometry firmware and ROS 2 driver for the Seeed XIAO nRF52840 Sense
- Added rotational camera and LED strip features, and coded firmware for operating them via serial commands

<hr>

## Challenges

- Calibrating and finetuning parameters for EKF-related configurations, including VESC and IMU odometry
- Hardware limitations with CPU and RAM power forced us to lower frequencies of data publishing and perform timed launching
- Integrating Nav2 for autonomous navigation, which was not able to be accomplished during this time

<hr>

## Final Project Videos

*Click the thumbnails to open the media.*

### Final Presentation Slides
[<img src="docs\media\presentation.jpg" width="300">](https://docs.google.com/presentation/d/100er0TgurMbT8JcgmrrsiwRIkXcWnn7BoJnauJ2WcTg/edit?usp=sharing)

### Demo Video
[<img src="docs\media\video.jpg" width="300">](https://drive.google.com/file/d/1kE3kcbYyW5jzt6wzRNw6U-6pjJctakA8/view?usp=sharing)

<hr>

## Documentation

- [Project Reproduction guide](docs/reproduction.md) if you are interested in reproducing our project.
- [ROS 2 testing guide](docs/ros2_testing.md) for camera, IMU, LiDAR, and SLAM/RViz2 checks.
- [Software Systems documentation](docs/software_systems.md) for learning about our software architectures and systems.
- [Robot Hardware documentation](docs/robot_hardware.md) for learning about our improved hardware components, including electrical and mechanical revamps.
- [Project Proposal](docs/proposal.pdf) for reading state-of-the-art methods, plans, and potential limitations.

<hr>

## Acknowledgements

Special thank you to Professor Truong Nguyen and Professor Jack Silberman for sponsoring and advising my project!

README.md Format, reference to [winter-2024-final-project-team-7](https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-7)

<hr>

## Contacts

* Evan Chou - e3chou@ucsd.edu | evan.chou@live.com | [LinkedIn](https://www.linkedin.com/in/evanjchou/)
