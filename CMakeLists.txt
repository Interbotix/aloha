cmake_minimum_required(VERSION 2.8.3)
project(aloha)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

install(
  PROGRAMS
    scripts/auto_record.sh
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/dual_side_teleop.py
    scripts/get_episode_len.py
    scripts/one_side_teleop.py
    scripts/record_episodes.py
    scripts/replay_and_record_episodes.py
    scripts/replay_episodes.py
    scripts/sleep.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
