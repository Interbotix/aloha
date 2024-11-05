#!/usr/bin/env python3

from aloha.robot_utils import (
    sleep_arms,
    torque_on,
    disable_gravity_compensation,
    load_yaml_file,
)
import argparse
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from pathlib import Path
from typing import Dict, Sequence


def main() -> None:
    """
    Main function to parse command-line arguments, initialize robot configurations,
    and send selected robots to their sleep positions.
    """
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
        required=True,
        help='Specify the robot configuration to use: aloha_solo, aloha_static, or aloha_mobile.'
    )
    args = argparser.parse_args()

    # Load robot configuration
    robot_base = args.robot

    base_path = Path(__file__).resolve().parent.parent / "config"

    config = load_yaml_file('robot', robot_base, base_path).get('robot', {})

    # Calculate time step for movement based on FPS from config
    dt = 1 / config.get('fps', 50)

    # Create a global ROS node
    node = create_interbotix_global_node('aloha')

    # Dictionary to store robots
    robots: Dict[str, InterbotixManipulatorXS] = {}

    # Initialize leader arms
    for leader in config.get('leader_arms', []):
        print(f"Initializing leader arm: {leader['name']}")
        robots[leader['name']] = InterbotixManipulatorXS(
            robot_model=leader['model'],
            robot_name=leader['name'],
            node=node,
            iterative_update_fk=False,
        )

    # Initialize follower arms
    for follower in config.get('follower_arms', []):
        print(f"Initializing follower arm: {follower['name']}")
        robots[follower['name']] = InterbotixManipulatorXS(
            robot_model=follower['model'],
            robot_name=follower['name'],
            node=node,
            iterative_update_fk=False,
        )

    # Perform robot startup actions
    robot_startup(node)

    # Disable gravity compensation for leader arms
    for name, bot in robots.items():
        if 'leader' in name:
            disable_gravity_compensation(bot)

    # Determine which bots to put to sleep (all if '--all' flag is set, else only followers)
    bots_to_sleep: Sequence[InterbotixManipulatorXS] = (
        list(robots.values()) if args.all else [
            bot for name, bot in robots.items() if 'follower' in name]
    )

    # Enable torque on selected bots
    for bot in bots_to_sleep:
        torque_on(bot)

    # Move selected bots to their sleep positions
    sleep_arms(bots_to_sleep, home_first=True, dt=dt)

    # Perform robot shutdown actions
    robot_shutdown(node)


if __name__ == "__main__":
    main()
