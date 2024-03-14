# Mobile ALOHA: Learning Bimanual Mobile Manipulation with Low-Cost Whole-Body Teleoperation

#### Project Website: https://mobile-aloha.github.io/

This codebase is forked from the [ALOHA repo](https://github.com/tonyzhaozh/aloha), and contains implementation for teleoperation and data collection with the Mobile ALOHA hardware.
To build ALOHA, follow the [Hardware Assembly Tutorial](https://docs.google.com/document/d/1_3yhWjodSNNYlpxkRCPIlvIAaQ76Nqk2wsqhnEVM6Dc) and the quick start guide below.
To train imitation learning algorithms, you would also need to install [ACT for Mobile ALOHA](https://github.com/MarkFzp/act-plus-plus) which is forked from [ACT](https://github.com/tonyzhaozh/act).

### Repo Structure
- [``config``](./config/): a config for each robot, designating the port they should bind to, more details in quick start guide.
- [``launch``](./launch): a ROS launch file for all 4 cameras and all 4 robots.
- [``scripts``](./scripts/): Python scripts for teleop and data collection
- [``src``](./src/): Python modules for teleop and data collection

## Quick start guide

### Software selection -- OS:

Currently tested and working configurations:
- :white_check_mark: Ubuntu 18.04 + ROS 1 noetic
- :white_check_mark: Ubuntu 20.04 + ROS 1 noetic

Ongoing testing (compatibility effort underway):
- :construction: ROS 2
- :construction: >= Ubuntu 22.04

### Software installation - ROS:
1. Install ROS and interbotix software following https://docs.trossenrobotics.com/interbotix_xsarms_docs/
2. This will create the directory ``~/interbotix_ws`` which contains ``src``.
3. git clone this repo inside ``~/interbotix_ws/src``
4. ``source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh``
5. ``sudo apt-get install ros-noetic-usb-cam && sudo apt-get install ros-noetic-cv-bridge``
6. run ``catkin_make`` inside ``~/interbotix_ws``, make sure the build is successful
7. go to ``~/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/arm.py``, find function ``publish_positions``.
   Change ``self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)`` to ``self.T_sb = None``.
   This prevents the code from calculating FK at every step which delays teleoperation.
### Hardware installation:

The goal of this section is to run ``roslaunch aloha 4arms_teleop.launch``, which starts
communication with 4 robots and 3 cameras. It should work after finishing the following steps:

Step 1: Connect 4 robots to the computer via USB, and power on. *Do not use extension cable or usb hub*.
- To check if the robot is connected, install dynamixel wizard [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
- Dynamixel wizard is a very helpful debugging tool that connects to individual motors of the robot. It allows
things such as rebooting the motor (very useful!), torque on/off, and sending commands.
However, it has no knowledge about the kinematics of the robot, so be careful about collisions.
The robot *will* collapse if motors are torque off i.e. there is no automatically engaged brakes in joints.
- Open Dynamixel wizard, go into ``options`` and select:
  - Protocol 2.0
  - All ports
  - 1000000 bps
  - ID range from 0-10
- Note: repeat above every time before you scan.
- Then hit ``Scan``. There should be 4 devices showing up, each with 9 motors.


- One issue that arises is the port each robot binds to can change over time, e.g. a robot that
is initially ``ttyUSB0`` might suddenly become ``ttyUSB5``. To resolve this, we bind each robot to a fixed symlink
port with the following mapping:
  - ``ttyDXL_leader_right``: right leader robot (leader: the robot that the operator would be holding)
  - ``ttyDXL_follower_right``: right follower robot (follower: the robot that performs the task)
  - ``ttyDXL_leader_left``: left leader robot
  - ``ttyDXL_follower_left``: left follower robot
- Take ``ttyDXL_leader_right``: right leader robot as an example:
  1. Find the port that the right leader robot is currently binding to, e.g. ``ttyUSB0``
  2. run ``udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial`` to obtain the serial number. Use the first one that shows up, the format should look similar to ``FT6S4DSP``.
  3. ``sudo vim /etc/udev/rules.d/99-fixed-interbotix-udev.rules`` and add the following line:

         SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL_leader_right"

  4. This will make sure the right leader robot is *always* binding to ``ttyDXL_leader_right``
  5. Repeat with the rest of 3 arms.
- To apply the changes, run ``sudo udevadm control --reload && sudo udevadm trigger``
- If successful, you should be able to find ``ttyDXL*`` in your ``/dev``

Step 2: Set max current for gripper motors
- Open Dynamixel Wizard, and select the wrist motor for follower arms. The name of it should be ```[ID:009] XM430-W350```
- Tip: the LED on the base of robot will flash when it is talking to Dynamixel Wizard. This will help determine which robot is selected.
- Find ``38 Current Limit``, enter ``300``, then hit ``save`` at the bottom.
- Repeat this for both follower robots.
- This limits the max current through gripper motors, to prevent overloading errors.

Step 3: Setup 3 cameras
- You may use usb hub here, but *maximum 2 cameras per hub for reasonable latency*.
- To make sure all 3 cameras are binding to a consistent port, similar steps are needed.
- Cameras are by default binding to ``/dev/video{0, 1, 2...}``, while we want to have symlinks ``{CAM_RIGHT_WRIST, CAM_LEFT_WRIST, CAM_HIGH}``
- Take ``CAM_RIGHT_WRIST`` as an example, and let's say it is now binding to ``/dev/video0``. run ``udevadm info --name=/dev/video0 --attribute-walk | grep serial`` to obtain it's serial. Use the first one that shows up, the format should look similar to ``0E1A2B2F`` (USB) or ``133323070632`` (RealSense).
- Note that for RealSense cameras, the SN is written on the case.
- Then ``sudo vim /etc/udev/rules.d/99-fixed-interbotix-udev.rules`` and add the following line:

      SUBSYSTEM=="video4linux", ATTRS{serial}=="<serial number here>", ATTR{index}=="0", ATTRS{idProduct}=="085c", ATTR{device/latency_timer}="1", SYMLINK+="CAM_RIGHT_WRIST"

- Repeat this for ``{CAM_LEFT_WRIST, CAM_HIGH}`` in additional to ``CAM_RIGHT_WRIST``
- To apply the changes, run ``sudo udevadm control --reload && sudo udevadm trigger``
- If successful, you should be able to find ``{CAM_RIGHT_WRIST, CAM_LEFT_WRIST, CAM_HIGH}`` in your ``/dev``

Step 4: Setup the Slate base

At this point, have a new terminal

```shell
conda deactivate # if conda shows up by default
source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh
roslaunch aloha 4arms_teleop.launch
```

If no error message is showing up, the computer should be successfully connected to all 3 cameras, all 4 robot arms, and the robot base.

#### Trouble shooting

- Make sure Dynamixel Wizard is disconnected, and no app is using the camera streams.
  - These will prevent ROS from connecting to those devices.

### Software Installation - Conda:

Create and set up the conda environment automatically using the following command:

```shell
conda env create -f environment.yaml
```

The conda environment can also be set up manually using the following commands:

```shell
conda create -n aloha python=3.8.10
conda activate aloha
pip install dm_control
pip install einops
pip install h5py
pip install matplotlib
pip install mujoco
pip install opencv-python
pip install packaging
pip install pexpect
pip install pyquaternion
pip install pyyaml
pip install rospkg
pip install torch
pip install torchvision
pip install tqdm
pip install wandb
```

### Testing Teleoperation

> [!NOTE]
> Before running the commands below, be sure to place all 4 robots in their sleep positions, and open leader robot's gripper.
> All robots will rise to a height that is easy for teleoperation.

Open two terminals and enter the following:

```shell
# Terminal 1
conda deactivate
source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh
roslaunch aloha 4arms_teleop.launch use_cameras:=false

# Terminal 2
conda activate aloha
cd ~/interbotix_ws/src/aloha/scripts
python3 dual_side_teleop.py
```

The teleoperation will start when the leader side grippers are closed.

## Example Usages

To set up a new terminal, run:
```shell
conda activate aloha
cd ~/interbotix_ws/src/aloha/scripts
```

The ``dual_side_teleop.py`` we ran is for testing teleoperation and has no data collection.
To collect data for an episode, run:

```shell
python3 record_episodes.py --dataset_dir <data save dir> --episode_idx 0
```

This will store a hdf5 file at ``<data save dir>``.
To change episode length and other params, edit ``constants.py`` directly.

To visualize the episode collected, run:

```shell
python3 visualize_episodes.py --dataset_dir <data save dir> --episode_idx 0
```

To replay the episode collected with real robot, run:

```shell
python3 replay_episodes.py --dataset_dir <data save dir> --episode_idx 0
```

To safely lower 4 robots before e.g. cutting off power, run:

```shell
python3 sleep.py
```
