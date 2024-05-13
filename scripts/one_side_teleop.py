#!/usr/bin/env python3

import sys
import time

from aloha.constants import (
    DT,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    LEADER2FOLLOWER_JOINT_FN,
    LEADER_GRIPPER_CLOSE_THRESH,
    LEADER_GRIPPER_JOINT_MID,
    START_ARM_POSE,
)
from aloha.robot_utils import (
    get_arm_gripper_positions,
    move_arms,
    move_grippers,
    torque_off,
    torque_on,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
import IPython
import rclpy

e = IPython.embed


def prep_robots(leader_bot: InterbotixManipulatorXS, follower_bot: InterbotixManipulatorXS):
    # reboot gripper motors, and set operating modes for all motors
    follower_bot.core.robot_reboot_motors('single', 'gripper', True)
    follower_bot.core.robot_set_operating_modes('group', 'arm', 'position')
    follower_bot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
    leader_bot.core.robot_set_operating_modes('group', 'arm', 'position')
    leader_bot.core.robot_set_operating_modes('single', 'gripper', 'position')
    follower_bot.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 300)
    torque_on(follower_bot)
    torque_on(leader_bot)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([leader_bot, follower_bot], [start_arm_qpos] * 2, move_time=1.0)
    # move grippers to starting position
    move_grippers(
        [leader_bot, follower_bot],
        [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE],
        move_time=0.5
    )


def press_to_start(leader_bot: InterbotixManipulatorXS):
    # press gripper to start data collection
    # disable torque for only gripper joint of leader robot to allow user movement
    leader_bot.core.robot_torque_enable('single', 'gripper', False)
    print('Close the gripper to start')
    pressed = False
    while rclpy.ok() and not pressed:
        gripper_pos = get_arm_gripper_positions(leader_bot)
        if gripper_pos < LEADER_GRIPPER_CLOSE_THRESH:
            pressed = True
        time.sleep(DT/10)
    torque_off(leader_bot)
    print('Started!')


def main(robot_side: str):
    follower_bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        robot_name=f'follower_{robot_side}',
        node_owner=True,
    )
    leader_bot = InterbotixManipulatorXS(
        robot_model='wx250s',
        robot_name=f'leader_{robot_side}',
        node_owner=False,
    )

    prep_robots(leader_bot, follower_bot)
    press_to_start(leader_bot)

    # Teleoperation loop
    gripper_command = JointSingleCommand(name='gripper')
    while rclpy.ok():
        # sync joint positions
        leader_state_joints = leader_bot.core.joint_states.position[:6]
        follower_bot.arm.set_joint_positions(leader_state_joints, blocking=False)
        # sync gripper positions
        leader_gripper_joint = leader_bot.core.joint_states.position[6]
        follower_gripper_joint_target = LEADER2FOLLOWER_JOINT_FN(leader_gripper_joint)
        gripper_command.cmd = follower_gripper_joint_target
        follower_bot.gripper.core.pub_single.publish(gripper_command)
        # sleep DT
        time.sleep(DT)


if __name__ == '__main__':
    side = sys.argv[1]
    if side not in ('left', 'right'):
        raise ValueError(f"Invalid side '{side}'. Must be 'left' or 'right'.")
    main(side)
