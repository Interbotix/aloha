#!/usr/bin/env python3

import argparse
from aloha.robot_utils import (
    enable_gravity_compensation,
    get_arm_gripper_positions,
    move_arms,
    move_grippers,
    torque_off,
    torque_on,
    load_yaml_file,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    LEADER2FOLLOWER_JOINT_FN,
    LEADER_GRIPPER_CLOSE_THRESH,
    LEADER_GRIPPER_JOINT_MID,
    START_ARM_POSE,
)

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from pathlib import Path
import rclpy
from rclpy.duration import Duration
from rclpy.constants import S_TO_NS
from typing import Dict




def opening_ceremony(robots: Dict[str, InterbotixManipulatorXS],
                     dt: float,
                     ) -> None:
    """
    Move all leader-follower pairs of robots to a starting pose for demonstration.

    :param robots: Dictionary containing robot instances categorized as 'leader' or 'follower'
    :param dt: Time interval (in seconds) for each movement step
    """
    # Separate leader and follower robots
    leader_bots = {name: bot for name,
                   bot in robots.items() if 'leader' in name}
    follower_bots = {name: bot for name,
                     bot in robots.items() if 'follower' in name}

    # Initialize an empty list to store matched pairs of leader and follower robots
    pairs = []

    # Create dictionaries mapping suffixes to leader and follower robots
    leader_suffixes = {name.split(
        '_', 1)[1]: bot for name, bot in leader_bots.items()}
    follower_suffixes = {name.split(
        '_', 1)[1]: bot for name, bot in follower_bots.items()}

    # Pair leader and follower robots based on matching suffixes
    for suffix, leader_bot in leader_suffixes.items():
        if suffix in follower_suffixes:
            # If matching follower exists, pair it with the leader
            follower_bot = follower_suffixes.pop(suffix)
            pairs.append((leader_bot, follower_bot))
        else:
            # Raise an error if there’s an unmatched leader suffix
            raise ValueError(
                f"Unmatched leader suffix '{suffix}' found. Every leader should have a corresponding follower with the same suffix.")

    # Check if any unmatched followers remain after pairing
    if follower_suffixes:
        unmatched_suffixes = ', '.join(follower_suffixes.keys())
        raise ValueError(
            f"Unmatched follower suffix(es) found: {unmatched_suffixes}. Every follower should have a corresponding leader with the same suffix.")

    # Ensure at least one leader-follower pair was created
    if not pairs:
        raise ValueError(
            "No valid leader-follower pairs found in the robot dictionary.")

    # Initialize each leader-follower pair
    for leader_bot, follower_bot in pairs:
        # Reboot gripper motors and set operating modes
        follower_bot.core.robot_reboot_motors('single', 'gripper', True)
        follower_bot.core.robot_set_operating_modes('group', 'arm', 'position')
        follower_bot.core.robot_set_operating_modes(
            'single', 'gripper', 'current_based_position')
        leader_bot.core.robot_set_operating_modes('group', 'arm', 'position')
        leader_bot.core.robot_set_operating_modes(
            'single', 'gripper', 'position')
        follower_bot.core.robot_set_motor_registers(
            'single', 'gripper', 'current_limit', 300)

        # Enable torque for leader and follower
        torque_on(follower_bot)
        torque_on(leader_bot)

        # Move arms to starting position
        start_arm_qpos = START_ARM_POSE[:6]
        move_arms(
            bot_list=[leader_bot, follower_bot],
            dt=dt,
            target_pose_list=[start_arm_qpos] * 2,
            moving_time=4.0,
        )

        # Move grippers to starting position
        move_grippers(
            [leader_bot, follower_bot],
            [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE],
            moving_time=0.5,
            dt=dt,
        )


def press_to_start(robots: Dict[str, InterbotixManipulatorXS],
                   dt: float,
                   gravity_compensation: bool,
                   ) -> None:
    """
    Wait for the user to close the grippers on all leader robots to start teleoperation.

    :param robots: Dictionary containing robot instances categorized as 'leader' or 'follower'
    :param dt: Time interval (in seconds) for each movement step
    :param gravity_compensation: Boolean flag to enable gravity compensation on leaders
    """
    # Extract leader bots from the robots dictionary
    leader_bots = {name: bot for name,
                   bot in robots.items() if 'leader' in name}

    # Disable torque for gripper joint of each leader bot to allow user movement
    for leader_bot in leader_bots.values():
        leader_bot.core.robot_torque_enable('single', 'gripper', False)

    print('Close the grippers to start')

    # Wait for the user to close the grippers on all leader robots
    pressed = False
    while rclpy.ok() and not pressed:
        pressed = all(
            get_arm_gripper_positions(leader_bot) < LEADER_GRIPPER_CLOSE_THRESH
            for leader_bot in leader_bots.values()
        )
        DT_DURATION = Duration(seconds=0, nanoseconds=dt * S_TO_NS)
        get_interbotix_global_node().get_clock().sleep_for(DT_DURATION)

    # Enable gravity compensation or turn off torque based on the parameter
    for leader_bot in leader_bots.values():
        if gravity_compensation:
            enable_gravity_compensation(leader_bot)
        else:
            torque_off(leader_bot)

    print('Started!')


def main(args: dict) -> None:
    """
    Main teleoperation setup function.

    :param args: Dictionary containing parsed arguments including gravity compensation
                 and robot configuration.
    """
    gravity_compensation = args.get('gravity_compensation', False)
    node = create_interbotix_global_node('aloha')

    # Load robot configuration
    robot_base = args.get('robot', '')

    # Base path of the config directory using absolute path
    base_path = Path(__file__).resolve().parent.parent / "config"

    config = load_yaml_file("robot", robot_base, base_path).get('robot', {})
    dt = 1 / config.get('fps', 30)

    # Initialize dictionary for robot instances
    robots = {}

    # Create leader arms from configuration
    for leader in config.get('leader_arms', []):
        robot_instance = InterbotixManipulatorXS(
            robot_model=leader['model'],
            robot_name=leader['name'],
            node=node,
            iterative_update_fk=False,
        )
        robots[leader['name']] = robot_instance

    # Create follower arms from configuration
    for follower in config.get('follower_arms', []):
        robot_instance = InterbotixManipulatorXS(
            robot_model=follower['model'],
            robot_name=follower['name'],
            node=node,
            iterative_update_fk=False,
        )
        robots[follower['name']] = robot_instance

    # Startup and initialize robot sequence
    robot_startup(node)
    opening_ceremony(robots, dt)
    press_to_start(robots, dt, gravity_compensation)

    # Define gripper command objects for each follower
    gripper_commands = {
        follower_name: JointSingleCommand(name='gripper') for follower_name in robots if 'follower' in follower_name
    }

    # Main teleoperation loop
    while rclpy.ok():
        for leader_name, leader_bot in robots.items():
            if 'leader' in leader_name:
                suffix = leader_name.replace('leader', '')
                follower_name = f'follower{suffix}'
                follower_bot = robots.get(follower_name)

                if follower_bot:
                    # Sync arm joint positions and gripper positions
                    leader_state_joints = leader_bot.arm.get_joint_positions()
                    follower_bot.arm.set_joint_positions(
                        leader_state_joints, blocking=False)

                    # Sync gripper positions
                    gripper_command = gripper_commands[follower_name]
                    gripper_command.cmd = LEADER2FOLLOWER_JOINT_FN(
                        leader_bot.gripper.get_gripper_position()
                    )
                    follower_bot.gripper.core.pub_single.publish(
                        gripper_command)

        # Sleep for the DT duration
        DT_DURATION = Duration(seconds=0, nanoseconds=dt * S_TO_NS)
        get_interbotix_global_node().get_clock().sleep_for(DT_DURATION)

    robot_shutdown(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-g', '--gravity_compensation',
        action='store_true',
        help='If set, gravity compensation will be enabled for the leader robots when teleop starts.',
    )
    parser.add_argument(
        '-r', '--robot',
        required=True,
        help='Specify the robot configuration to use: aloha_solo, aloha_static, or aloha_mobile.'
    )
    main(vars(parser.parse_args()))
