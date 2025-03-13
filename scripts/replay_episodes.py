#!/usr/bin/env python3

import argparse
import os
import time
from typing import Dict

import h5py
from aloha.real_env import make_real_env
from aloha.robot_utils import (
    move_grippers,
    load_yaml_file,
    JOINT_NAMES,
    FOLLOWER_GRIPPER_JOINT_OPEN,
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from pathlib import Path


# Define joint and gripper state names for tracking purposes
STATE_NAMES = JOINT_NAMES + ['gripper', 'left_finger', 'right_finger']


def main(args: Dict[str, any]) -> None:
    """
    Main function to replay a saved episode for the robot based on configuration parameters.
    Loads actions from an HDF5 file and applies them in a real environment.

    :param args: Dictionary of command-line arguments, including:
        - 'dataset_dir' (str): Path to the directory containing episode datasets.
        - 'episode_idx' (int): Index of the episode file to load.
        - 'robot' (str): Robot configuration name (e.g., 'aloha_solo', 'aloha_stationary', 'aloha_mobile').
    """
    # Load robot configuration
    robot_base = args.get('robot', '')

    base_path = Path(__file__).resolve().parent.parent / "config"

    config = load_yaml_file('robot', robot_base, base_path).get('robot', {})
    is_mobile = config.get('base', False)

    # Set the timestep duration for the environment update frequency
    dt = 1 / config.get('fps', 50)

    # Construct dataset path
    dataset_dir = args['dataset_dir']
    episode_idx = args['episode_idx']
    dataset_name = f'episode_{episode_idx}'
    dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')

    # Check if dataset exists, and exit if not
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    # Load actions from the dataset
    with h5py.File(dataset_path, 'r') as root:
        actions = root['/action'][()]
        base_actions = root['/base_action'][()] if is_mobile else None

    # Initialize the ROS node and create the real environment
    node = create_interbotix_global_node('aloha')
    env = make_real_env(node, setup_robots=False,
                        setup_base=is_mobile, config=config)

    # Set mobile base motor torque if applicable
    if is_mobile:
        env.base.base.set_motor_torque(True)
    robot_startup(node)

    # Configure and initialize follower robots
    for name, bot in env.robots.items():
        if 'follower' in name:
            bot.core.robot_reboot_motors('single', 'gripper', True)
            bot.core.robot_set_operating_modes('group', 'arm', 'position')
            bot.core.robot_set_operating_modes(
                'single', 'gripper', 'current_based_position')
            bot.core.robot_torque_enable('group', 'arm', True)
            bot.core.robot_torque_enable('single', 'gripper', True)

    # Reset the environment to initial state
    env.reset()
    time0 = time.time()

    # Execute each action in the episode
    if is_mobile:
        for action, base_action in zip(actions, base_actions):
            time1 = time.time()
            env.step(action, base_action)
            time.sleep(max(0, dt - (time.time() - time1)))
    else:
        for action in actions:
            time1 = time.time()
            env.step(action, None)
            time.sleep(max(0, dt - (time.time() - time1)))

    # Print average frames per second
    print(f'Avg fps: {len(actions) / (time.time() - time0)}')

    # Collect follower robots and set gripper positions
    follower_bots = [bot for name, bot in env.robots.items()
                     if 'follower' in name]
    gripper_positions = [FOLLOWER_GRIPPER_JOINT_OPEN] * len(follower_bots)

    # Move follower grippers to open position
    move_grippers(follower_bots, gripper_positions, moving_time=0.5, dt=dt)
    robot_shutdown(node)


if __name__ == '__main__':
    # Define command-line arguments
    parser = argparse.ArgumentParser(
        description="Replays a saved episode for the robot.")

    parser.add_argument(
        '--dataset_dir',
        action='store',
        type=str,
        help='Path to the directory containing the dataset.',
        required=True,
    )
    parser.add_argument(
        '--episode_idx',
        action='store',
        type=int,
        help='Index of the episode to replay.',
        default=0,
        required=False,
    )
    parser.add_argument(
        '-r', '--robot',
        action='store',
        type=str,
        help='Robot configuration name (e.g., aloha_solo, aloha_stationary, aloha_mobile).',
        required=True,
    )

    # Execute main function with parsed arguments
    main(vars(parser.parse_args()))
