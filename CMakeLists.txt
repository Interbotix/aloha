cmake_minimum_required(VERSION 3.5)
project(aloha)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
    PROGRAMS
      scripts/dual_side_teleop.py
      scripts/get_episode_len.py
      scripts/one_side_teleop.py
      scripts/record_episodes.py
      scripts/replay_and_record_episodes.py
      scripts/replay_episodes.py
      scripts/sleep.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
