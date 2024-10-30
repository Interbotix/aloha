#!/usr/bin/env python3

import argparse
import os
import time

from aloha.real_env import (
    make_real_env,
)
from aloha.robot_utils import (
    move_grippers,
    load_yaml_file,
    JOINT_NAMES,
    FOLLOWER_GRIPPER_JOINT_OPEN,
)
import h5py

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)

STATE_NAMES = JOINT_NAMES + ['gripper', 'left_finger', 'right_finger']

def main(args):

    robot_base = args.get('robot', '')

    config = load_yaml_file('robot', robot_base)

    is_mobile = config.get('base', False)

    DT = 1 / config.get('fps', 50)
    
    dataset_dir = args['dataset_dir']
    episode_idx = args['episode_idx']
    dataset_name = f'episode_{episode_idx}'

    dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        actions = root['/action'][()]
        if is_mobile:
            base_actions = root['/base_action'][()]

    node = create_interbotix_global_node('aloha')

    env = make_real_env(node, setup_robots=False, setup_base=is_mobile, config=config)
    
    if is_mobile:
        env.base.base.set_motor_torque(True)
    robot_startup(node)

    for name, bot in env.robots.items():
        if 'follower' in name:
            # Reboot gripper motors for each follower bot
            bot.core.robot_reboot_motors('single', 'gripper', True)
            # Set the operating mode of the arm to 'position'
            bot.core.robot_set_operating_modes('group', 'arm', 'position')
            # Set the operating mode of the gripper to 'current_based_position'
            bot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
            # Enable torque for the robot
            bot.core.robot_torque_enable('group', 'arm', True)
            bot.core.robot_torque_enable('single', 'gripper', True)

    env.reset()

    time0 = time.time()
    
    if is_mobile:
        for action, base_action in zip(actions, base_actions):
            time1 = time.time()
            env.step(action, base_action)
            time.sleep(max(0, DT - (time.time() - time1)))
    else:
        for action in actions:
            time1 = time.time()
            env.step(action, None)
            time.sleep(max(0, DT - (time.time() - time1)))
    print(f'Avg fps: {len(actions) / (time.time() - time0)}')

    # Create a list to hold all follower bots and their target gripper positions
    follower_bots = []
    gripper_positions = []

    # Iterate through the robots dictionary and collect follower bots
    for name, bot in env.robots.items():
        if 'follower' in name:
            follower_bots.append(bot)
            # Set the gripper position dynamically (e.g., to FOLLOWER_GRIPPER_JOINT_OPEN)
            gripper_positions.append(FOLLOWER_GRIPPER_JOINT_OPEN)

    # Call move_grippers on the dynamically collected follower bots and gripper positions
    move_grippers(follower_bots, gripper_positions, moving_time=0.5, DT=DT)
    robot_shutdown(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--dataset_dir',
        action='store',
        type=str,
        help='Dataset dir.',
        required=True,
    )
    parser.add_argument(
        '--episode_idx',
        action='store',
        type=int,
        help='Episode index.',
        default=0,
        required=False,
    )

    parser.add_argument(
        '-r', '--robot',
        action='store',
        type=str,
        help='Robot Setup.',
        required=True,
    )

    main(vars(parser.parse_args()))
