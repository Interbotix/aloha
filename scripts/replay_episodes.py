#!/usr/bin/env python3

import argparse
import os
import time

from aloha.constants import (
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FPS,
    IS_MOBILE,
    JOINT_NAMES,
)
from aloha.real_env import (
    make_real_env,
)
from aloha.robot_utils import (
    move_grippers,
)
import h5py
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)

STATE_NAMES = JOINT_NAMES + ['gripper', 'left_finger', 'right_finger']


def main(args):
    dataset_dir = args['dataset_dir']
    episode_idx = args['episode_idx']
    dataset_name = f'episode_{episode_idx}'

    dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        actions = root['/action'][()]
        if IS_MOBILE:
            base_actions = root['/base_action'][()]

    node = create_interbotix_global_node('aloha')

    env = make_real_env(node, setup_robots=False, setup_base=IS_MOBILE)

    if IS_MOBILE:
        env.base.base.set_motor_torque(True)
    robot_startup(node)

    env.setup_robots()

    env.reset()

    time0 = time.time()
    DT = 1 / FPS
    if IS_MOBILE:
        for action, base_action in zip(actions, base_actions):
            time1 = time.time()
            env.step(action, base_action, get_base_vel=True)
            time.sleep(max(0, DT - (time.time() - time1)))
    else:
        for action in actions:
            time1 = time.time()
            env.step(action, None, get_base_vel=False)
            time.sleep(max(0, DT - (time.time() - time1)))
    print(f'Avg fps: {len(actions) / (time.time() - time0)}')

    # open
    move_grippers(
        [env.follower_bot_left, env.follower_bot_right],
        [FOLLOWER_GRIPPER_JOINT_OPEN] * 2,
        moving_time=0.5,
    )
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
    main(vars(parser.parse_args()))
