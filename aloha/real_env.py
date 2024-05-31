import collections
import time

from aloha.constants import (
    DT,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN,
    FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN,
    FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN,
    IS_MOBILE,
    LEADER_GRIPPER_JOINT_NORMALIZE_FN,
    START_ARM_POSE,
)
from aloha.robot_utils import (
    ImageRecorder,
    move_arms,
    move_grippers,
    Recorder,
    setup_follower_bot,
    setup_leader_bot,
)
import dm_env
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    InterbotixRobotNode,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.slate import InterbotixSlate
from interbotix_xs_msgs.msg import JointSingleCommand
import matplotlib.pyplot as plt
import numpy as np


class RealEnv:
    """
    Environment for real robot bi-manual manipulation.

    Action space: [
        left_arm_qpos (6),             # absolute joint position
        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
        right_arm_qpos (6),            # absolute joint position
        right_gripper_positions (1),   # normalized gripper position (0: close, 1: open)
    ]

    Observation space: {
        "qpos": Concat[
            left_arm_qpos (6),          # absolute joint position
            left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
            right_arm_qpos (6),         # absolute joint position
            right_gripper_qpos (1)      # normalized gripper position (0: close, 1: open)
        ]
        "qvel": Concat[
            left_arm_qvel (6),          # absolute joint velocity (rad)
            left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
            right_arm_qvel (6),         # absolute joint velocity (rad)
            right_gripper_qvel (1)      # normalized gripper velocity (pos: opening, neg: closing)
        ]
        "images": {
            "cam_high": (480x640x3),        # h, w, c, dtype='uint8'
            "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
            "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
            "cam_right_wrist": (480x640x3)  # h, w, c, dtype='uint8'
        }
    """

    def __init__(
        self,
        node: InterbotixRobotNode,
        setup_robots: bool = True,
        setup_base: bool = False,
        is_mobile: bool = IS_MOBILE,
        torque_base: bool = False,
    ):
        """Initialize the Real Robot Environment

        :param node: The InterbotixRobotNode to build the Interbotix API on
        :param setup_robots: True to run through the arm setup process on init, defaults to True
        :param setup_base: True to run through the base setup process on init, defaults to False
        :param is_mobile: True to initialize the Mobile ALOHA environment, False for the Stationary
            ALOHA environment, defaults to IS_MOBILE
        :param torque_base: True to torque the base on after setup, False otherwise, defaults to
            True. Only applies when IS_MOBILE is True
        :raises ValueError: On providing False for setup_base but the robot is not mobile
        """
        self.follower_bot_left = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            robot_name='follower_left',
            node=node,
            iterative_update_fk=False,
        )
        self.follower_bot_right = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            robot_name='follower_right',
            node=node,
            iterative_update_fk=False,
        )

        self.recorder_left = Recorder('left', node=node)
        self.recorder_right = Recorder('right', node=node)
        self.image_recorder = ImageRecorder(node=node, is_mobile=IS_MOBILE)
        self.gripper_command = JointSingleCommand(name='gripper')

        if setup_robots:
            self.setup_robots()

        if setup_base:
            if is_mobile:
                self.setup_base(node, torque_base)
            else:
                raise ValueError((
                    'Requested to set up base but robot is not mobile. '
                    "Hint: check the 'IS_MOBILE' constant."
                ))

    def setup_base(self, node: InterbotixRobotNode, torque_enable: bool = False):
        """Create and configure the SLATE base node

        :param node: The InterbotixRobotNode to build the SLATE base module on
        :param torque_enable: True to torque the base on setup, defaults to False
        """
        self.base = InterbotixSlate(
            'aloha',
            node=node,
        )
        self.base.base.set_motor_torque(torque_enable)

    def setup_robots(self):
        setup_follower_bot(self.follower_bot_left)
        setup_follower_bot(self.follower_bot_right)

    def get_qpos(self):
        left_qpos_raw = self.recorder_left.qpos
        right_qpos_raw = self.recorder_right.qpos
        left_arm_qpos = left_qpos_raw[:6]
        right_arm_qpos = right_qpos_raw[:6]
        left_gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(left_qpos_raw[7])]
        right_gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(right_qpos_raw[7])]
        return np.concatenate(
            [left_arm_qpos, left_gripper_qpos, right_arm_qpos, right_gripper_qpos]
        )

    def get_qvel(self):
        left_qvel_raw = self.recorder_left.qvel
        right_qvel_raw = self.recorder_right.qvel
        left_arm_qvel = left_qvel_raw[:6]
        right_arm_qvel = right_qvel_raw[:6]
        left_gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(left_qvel_raw[7])]
        right_gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(right_qvel_raw[7])]
        return np.concatenate(
            [left_arm_qvel, left_gripper_qvel, right_arm_qvel, right_gripper_qvel]
        )

    def get_effort(self):
        left_effort_raw = self.recorder_left.effort
        right_effort_raw = self.recorder_right.effort
        left_robot_effort = left_effort_raw[:7]
        right_robot_effort = right_effort_raw[:7]
        return np.concatenate([left_robot_effort, right_robot_effort])

    def get_images(self):
        return self.image_recorder.get_images()

    def get_base_vel(self):
        linear_vel = self.base.base.get_linear_velocity().x
        angular_vel = self.base.base.get_angular_velocity().z
        return np.array([linear_vel, angular_vel])

    def set_gripper_pose(
        self,
        left_gripper_desired_pos_normalized,
        right_gripper_desired_pos_normalized
    ):
        left_gripper_desired_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(
            left_gripper_desired_pos_normalized
        )
        self.gripper_command.cmd = left_gripper_desired_joint
        self.follower_bot_left.gripper.core.pub_single.publish(self.gripper_command)

        right_gripper_desired_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(
            right_gripper_desired_pos_normalized
        )
        self.gripper_command.cmd = right_gripper_desired_joint
        self.follower_bot_right.gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]
        move_arms(
            [self.follower_bot_left, self.follower_bot_right],
            [reset_position, reset_position],
            moving_time=1.0,
        )

    def _reset_gripper(self):
        """
        Set to position mode and do position resets.

        First open then close, then change back to PWM mode
        """
        move_grippers(
            [self.follower_bot_left, self.follower_bot_right],
            [FOLLOWER_GRIPPER_JOINT_OPEN] * 2,
            moving_time=0.5,
        )
        move_grippers(
            [self.follower_bot_left, self.follower_bot_right],
            [FOLLOWER_GRIPPER_JOINT_CLOSE] * 2,
            moving_time=1.0,
        )

    def get_observation(self, get_base_vel=False):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        obs['base_vel'] = self.get_base_vel()
        if get_base_vel:
            obs['base_vel'] = self.get_base_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            # Reboot follower robot gripper motors
            self.follower_bot_left.core.robot_reboot_motors('single', 'gripper', True)
            self.follower_bot_right.core.robot_reboot_motors('single', 'gripper', True)
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )

    def step(self, action, base_action=None, get_base_vel=False, get_obs=True):
        state_len = int(len(action) / 2)
        left_action = action[:state_len]
        right_action = action[state_len:]
        self.follower_bot_left.arm.set_joint_positions(left_action[:6], blocking=False)
        self.follower_bot_right.arm.set_joint_positions(right_action[:6], blocking=False)
        self.set_gripper_pose(left_action[-1], right_action[-1])
        if base_action is not None:
            base_action_linear, base_action_angular = base_action
            self.base.base.command_velocity_xyaw(x=base_action_linear, yaw=base_action_angular)
        if get_obs:
            obs = self.get_observation(get_base_vel)
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs)


def get_action(
    leader_bot_left: InterbotixManipulatorXS,
    leader_bot_right: InterbotixManipulatorXS
):
    action = np.zeros(14)  # 6 joint + 1 gripper, for two arms
    # Arm actions
    action[:6] = leader_bot_left.core.joint_states.position[:6]
    action[7:7+6] = leader_bot_right.core.joint_states.position[:6]
    # Gripper actions
    action[6] = LEADER_GRIPPER_JOINT_NORMALIZE_FN(leader_bot_left.core.joint_states.position[6])
    action[7+6] = LEADER_GRIPPER_JOINT_NORMALIZE_FN(leader_bot_right.core.joint_states.position[6])

    return action


def make_real_env(
    node: InterbotixRobotNode = None,
    setup_robots: bool = True,
    setup_base: bool = False,
    torque_base: bool = False,
):
    if node is None:
        node = get_interbotix_global_node()
        if node is None:
            node = create_interbotix_global_node('aloha')
    env = RealEnv(
        node=node,
        setup_robots=setup_robots,
        setup_base=setup_base,
        is_mobile=IS_MOBILE,
        torque_base=torque_base,
    )
    return env


def test_real_teleop():
    """
    Test bimanual teleoperation and show image observations onscreen.

    It first reads joint poses from both leader arms.
    Then use it as actions to step the environment.
    The environment returns full observations including images.

    An alternative approach is to have separate scripts for teleop and observation recording.
    This script will result in higher fidelity (obs, action) pairs
    """
    onscreen_render = True
    render_cam = 'cam_left_wrist'

    node = get_interbotix_global_node()

    # source of data
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
    setup_leader_bot(leader_bot_left)
    setup_leader_bot(leader_bot_right)

    # environment setup
    env = make_real_env(node=node)
    ts = env.reset(fake=True)
    episode = [ts]
    # visualization setup
    if onscreen_render:
        ax = plt.subplot()
        plt_img = ax.imshow(ts.observation['images'][render_cam])
        plt.ion()

    for _ in range(1000):
        action = get_action(leader_bot_left, leader_bot_right)
        ts = env.step(action)
        episode.append(ts)

        if onscreen_render:
            plt_img.set_data(ts.observation['images'][render_cam])
            plt.pause(DT)
        else:
            time.sleep(DT)


if __name__ == '__main__':
    test_real_teleop()
