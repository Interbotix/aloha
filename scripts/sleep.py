#!/usr/bin/env python3

import argparse

from aloha.robot_utils import (
    move_arms,
    torque_on
)
from interbotix_xs_modules.arm import InterbotixManipulatorXS


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--all', action='store_true', default=False)
    args = argparser.parse_args()

    follower_bot_left = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='follower_left',
        init_node=True,
    )
    follower_bot_right = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='follower_right',
        init_node=False,
    )
    leader_bot_left = InterbotixManipulatorXS(
        robot_model='wx250s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='leader_left',
        init_node=False,
    )
    leader_bot_right = InterbotixManipulatorXS(
        robot_model='wx250s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='leader_right',
        init_node=False,
    )

    all_bots = [follower_bot_left, follower_bot_right, leader_bot_left, leader_bot_right]
    follower_bots = [follower_bot_left, follower_bot_right]
    bots_to_sleep = all_bots if args.all else follower_bots

    follower_sleep_position = (0., -1.7, 1.55, 0., 0.65, 0.)
    leader_sleep_left_position = (0., 0., 0., 0., 0., 0.)
    leader_sleep_right_position = (0., 0., 0., 0., 0., 0.)

    positions_to_sleep_followers = [follower_sleep_position] * 2
    positions_to_sleep_all = (
        positions_to_sleep_followers + [leader_sleep_left_position, leader_sleep_right_position]
    )
    positions_to_sleep = positions_to_sleep_all if args.all else positions_to_sleep_followers

    leader_bots = [leader_bot_left, leader_bot_right]
    for bot in bots_to_sleep:
        torque_on(bot)

    move_arms(bots_to_sleep, positions_to_sleep, move_time=2.0)

    if args.all:
        safe_sleep = (0., -1.80, 1.55, 0., -1.57, 0.)
        move_arms(leader_bots, [safe_sleep] * 2, move_time=2.0)


if __name__ == '__main__':
    main()
