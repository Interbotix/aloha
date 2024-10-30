#!/usr/bin/env python3

import argparse
import yaml
from aloha.robot_utils import (
    sleep_arms,
    torque_on,
    disable_gravity_compensation,
    load_yaml_file
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import os



def main():
    # Parse command-line arguments
    argparser = argparse.ArgumentParser(
        prog='sleep',
        description='Sends arms to their sleep poses',
    )
    argparser.add_argument(
        '-a', '--all',
        help='If set, also sleeps leader arms',
        action='store_true',
        default=False,
    )
    argparser.add_argument(
        '-r', '--robot',
        choices=['aloha_solo', 'aloha_static', 'aloha_mobile'],
        required=True,
        help='Specify the robot configuration to use: aloha_solo, aloha_static, or aloha_mobile.'
    )

    args = argparser.parse_args()

    robot_base = args.robot

    config = load_yaml_file('robot',robot_base)

    DT = 1/config.get('fps', 50)

    # Create a global ROS node
    node = create_interbotix_global_node('aloha')

    # Create a dictionary to store robots
    robots = {}

    # Create leader arms
    for leader in config.get('leader_arms', []):
        print(leader['name'])
        robot_instance = InterbotixManipulatorXS(
            robot_model=leader['model'],
            robot_name=leader['name'],
            node=node,
            iterative_update_fk=False,
        )
        robots[leader['name']] = robot_instance

    # Create follower arms
    for follower in config.get('follower_arms', []):
        print(follower['name'])
        robot_instance = InterbotixManipulatorXS(
            robot_model=follower['model'],
            robot_name=follower['name'],
            node=node,
            iterative_update_fk=False,
        )
        robots[follower['name']] = robot_instance

    # Perform robot startup actions
    robot_startup(node)

    # Disable gravity compensation for leaders if applicable
    for name, bot in robots.items():
        if 'leader' in name:
            disable_gravity_compensation(bot)

    # Determine which bots to put to sleep
    bots_to_sleep = robots.values() if args.all else [bot for name, bot in robots.items() if 'follower' in name]

    # Enable torque on selected bots
    for bot in bots_to_sleep:
        torque_on(bot)

    # Move selected bots to their sleep positions
    sleep_arms(bots_to_sleep, home_first=True, DT=DT)

    # Perform robot shutdown actions
    robot_shutdown(node)


if __name__ == "__main__":
    main()

