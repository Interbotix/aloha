# Interbotix ALOHA

Project Websites:

* [ALOHA](https://tonyzhaozh.github.io/aloha/)
* [Mobile ALOHA](https://mobile-aloha.github.io/)

This codebase is forked from the [Mobile ALOHA repo](https://github.com/MarkFzp/mobile-aloha), and contains teleoperation and dataset collection and evaluation tools for the Stationary and Mobile ALOHA kits available from Trossen Robotics.

To get started with your ALOHA kit, follow the [ALOHA Getting Started Documentation](https://docs.trossenrobotics.com/aloha_docs/getting_started.html).

To train imitation learning algorithms, you would also need to install:

* [ACT for Stationary ALOHA](https://github.com/tonyzhaozh/act).
* [ACT++ for Mobile ALOHA](https://github.com/MarkFzp/act-plus-plus)

# Structure
- [``aloha``](./aloha/): Python package providing useful classes and constants for teleoperation and dataset collection.
- [``config``](./config/): a config for each robot, designating the port they should bind to, more details in quick start guide.
- [``launch``](./launch): a ROS 2 launch file for all cameras and manipulators.
- [``scripts``](./scripts/): Python scripts for teleop and data collection
