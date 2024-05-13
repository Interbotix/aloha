#!/usr/bin/env python3

import argparse

from aloha.robot_utils import (
    torque_on
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--all', action='store_true', default=False)
    args = argparser.parse_args()

    node = create_interbotix_global_node('aloha')

    follower_bot_left = InterbotixManipulatorXS(
        robot_model='vx300s',
        robot_name='follower_left',
        node=node,
    )
    follower_bot_right = InterbotixManipulatorXS(
        robot_model='vx300s',
        robot_name='follower_right',
        node=node,
    )
    leader_bot_left = InterbotixManipulatorXS(
        robot_model='wx250s',
        robot_name='leader_left',
        node=node,
    )
    leader_bot_right = InterbotixManipulatorXS(
        robot_model='wx250s',
        robot_name='leader_right',
        node=node,
    )

    robot_startup(node)

    all_bots = [follower_bot_left, follower_bot_right, leader_bot_left, leader_bot_right]
    follower_bots = [follower_bot_left, follower_bot_right]
    bots_to_sleep = all_bots if args.all else follower_bots

    for bot in bots_to_sleep:
        torque_on(bot)

    for bot in bots_to_sleep:
        bot.arm.go_to_sleep_pose(moving_time=2.0, accel_time=0.3)

    robot_shutdown(node)


if __name__ == '__main__':
    main()
