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

    puppet_bot_left = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='puppet_left',
        init_node=True,
    )
    puppet_bot_right = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='puppet_right',
        init_node=False,
    )
    master_bot_left = InterbotixManipulatorXS(
        robot_model='wx250s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='master_left',
        init_node=False,
    )
    master_bot_right = InterbotixManipulatorXS(
        robot_model='wx250s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='master_right',
        init_node=False,
    )

    all_bots = [puppet_bot_left, puppet_bot_right, master_bot_left, master_bot_right]
    puppet_bots = [puppet_bot_left, puppet_bot_right]
    bots_to_sleep = all_bots if args.all else puppet_bots

    puppet_sleep_position = (0., -1.7, 1.55, 0., 0.65, 0.)
    master_sleep_left_position = (-0.61, 0., 0.43, 0., 1.04, -0.65)
    master_sleep_right_position = (0.61, 0., 0.43, 0., 1.04, 0.65)

    positions_to_sleep_puppets = [puppet_sleep_position] * 2
    positions_to_sleep_all = (
        positions_to_sleep_puppets + [master_sleep_left_position, master_sleep_right_position]
    )
    positions_to_sleep = positions_to_sleep_all if args.all else positions_to_sleep_puppets

    master_bots = [master_bot_left, master_bot_right]
    for bot in bots_to_sleep:
        torque_on(bot)

    move_arms(bots_to_sleep, positions_to_sleep, move_time=2.0)

    if args.all:
        master_sleep_left_position_2 = (0., 0.66, -0.27, -0.0, 1.1, 0)
        master_sleep_right_position_2 = (0., 0.66, -0.27, -0.0, 1.1, 0)
        move_arms(
            master_bots,
            [master_sleep_left_position_2, master_sleep_right_position_2],
            move_time=1.0,
        )


if __name__ == '__main__':
    main()
