#!/usr/bin/env python3

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
import rclpy


def opening_ceremony(
    leader_bot_left: InterbotixManipulatorXS,
    leader_bot_right: InterbotixManipulatorXS,
    follower_bot_left: InterbotixManipulatorXS,
    follower_bot_right: InterbotixManipulatorXS,
) -> None:
    """Move all 4 robots to a pose where it is easy to start demonstration."""
    # reboot gripper motors, and set operating modes for all motors
    follower_bot_left.core.robot_reboot_motors('single', 'gripper', True)
    follower_bot_left.core.robot_set_operating_modes('group', 'arm', 'position')
    follower_bot_left.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
    leader_bot_left.core.robot_set_operating_modes('group', 'arm', 'position')
    leader_bot_left.core.robot_set_operating_modes('single', 'gripper', 'position')
    follower_bot_left.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 300)

    follower_bot_right.core.robot_reboot_motors('single', 'gripper', True)
    follower_bot_right.core.robot_set_operating_modes('group', 'arm', 'position')
    follower_bot_right.core.robot_set_operating_modes(
        'single', 'gripper', 'current_based_position'
    )
    leader_bot_right.core.robot_set_operating_modes('group', 'arm', 'position')
    leader_bot_right.core.robot_set_operating_modes('single', 'gripper', 'position')
    follower_bot_left.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 300)

    torque_on(follower_bot_left)
    torque_on(leader_bot_left)
    torque_on(follower_bot_right)
    torque_on(leader_bot_right)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms(
        [leader_bot_left, follower_bot_left, leader_bot_right, follower_bot_right],
        [start_arm_qpos] * 4,
        move_time=4.0,
    )
    # move grippers to starting position
    move_grippers(
        [leader_bot_left, follower_bot_left, leader_bot_right, follower_bot_right],
        [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE] * 2,
        move_time=0.5
    )


def press_to_start(
    leader_bot_left: InterbotixManipulatorXS,
    leader_bot_right: InterbotixManipulatorXS,
) -> None:
    # press gripper to start teleop
    # disable torque for only gripper joint of leader robot to allow user movement
    leader_bot_left.core.robot_torque_enable('single', 'gripper', False)
    leader_bot_right.core.robot_torque_enable('single', 'gripper', False)
    print('Close the gripper to start')
    pressed = False
    while rclpy.ok() and not pressed:
        gripper_pos_left = get_arm_gripper_positions(leader_bot_left)
        gripper_pos_right = get_arm_gripper_positions(leader_bot_right)
        pressed = (
            (gripper_pos_left < LEADER_GRIPPER_CLOSE_THRESH) and
            (gripper_pos_right < LEADER_GRIPPER_CLOSE_THRESH)
        )
        time.sleep(DT/10.0)
    torque_off(leader_bot_left)
    torque_off(leader_bot_right)
    print('Started!')


def main() -> None:
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

    opening_ceremony(
        leader_bot_left,
        leader_bot_right,
        follower_bot_left,
        follower_bot_right,
    )
    press_to_start(leader_bot_left, leader_bot_right)

    # Teleoperation loop
    gripper_left_command = JointSingleCommand(name='gripper')
    gripper_right_command = JointSingleCommand(name='gripper')
    while rclpy.ok():
        # sync joint positions
        leader_left_state_joints = leader_bot_left.core.joint_states.position[:6]
        leader_right_state_joints = leader_bot_right.core.joint_states.position[:6]
        follower_bot_left.arm.set_joint_positions(leader_left_state_joints, blocking=False)
        follower_bot_right.arm.set_joint_positions(leader_right_state_joints, blocking=False)
        # sync gripper positions
        gripper_left_command.cmd = LEADER2FOLLOWER_JOINT_FN(
            leader_bot_left.core.joint_states.position[6]
        )
        gripper_right_command.cmd = LEADER2FOLLOWER_JOINT_FN(
            leader_bot_right.core.joint_states.position[6]
        )
        follower_bot_left.gripper.core.pub_single.publish(gripper_left_command)
        follower_bot_right.gripper.core.pub_single.publish(gripper_right_command)
        # sleep DT
        time.sleep(DT)


if __name__ == '__main__':
    main()