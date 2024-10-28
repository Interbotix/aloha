import collections
import importlib

import time


from aloha.robot_utils import (
    ImageRecorder,
    move_arms,
    move_grippers,
    setup_follower_bot,
    setup_leader_bot,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN,
    FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN,
    FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN,
    LEADER_GRIPPER_JOINT_NORMALIZE_FN,
    START_ARM_POSE,
    DT
)

import dm_env
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    InterbotixRobotNode,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

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
        torque_base: bool = False,
        config: dict = None
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
        self.is_mobile = config.get('base',False)
        print(f"IS MObile is {self.is_mobile}")
        # Dynamically import module based on config value
        if self.is_mobile:
            print("Importing Slate")
            xs_robot_module = importlib.import_module('interbotix_xs_modules.xs_robot.slate')
            self.InterbotixSlate = getattr(xs_robot_module, 'InterbotixSlate')
        
        # Dictionary to store the robot instances
        self.robots = {}

        # Iterate through leader arms from the YAML
        for leader in config.get('leader_arms', []):
            self.robots[leader['name']] = InterbotixManipulatorXS(
                robot_model=leader['model'],
                robot_name=leader['name'],
                node=node,
                iterative_update_fk=False,
            )

        # Iterate through follower arms from the YAML
        for follower in config.get('follower_arms', []):
            self.robots[follower['name']] = InterbotixManipulatorXS(
                robot_model=follower['model'],
                group_name='arm',   # Assuming this remains the same for all followers
                gripper_name='gripper',   # Assuming this remains the same for all followers
                robot_name=follower['name'],
                node=node,
                iterative_update_fk=False,
            )

        self.is_mobile = config.get('base', False)

        self.image_recorder = ImageRecorder(node=node, config=config)
        self.gripper_command = JointSingleCommand(name='gripper')

        if setup_robots:
            self.setup_robots()

        if setup_base:
            if self.is_mobile:
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
        self.base = self.InterbotixSlate(
            'aloha',
            node=node,
        )
        self.base.base.set_motor_torque(torque_enable)

    def setup_robots(self):
        setup_follower_bot(self.follower_bot_left)
        setup_follower_bot(self.follower_bot_right)
    
    def get_qpos(self):
        # Initialize a list to hold the arm and gripper positions
        qpos_list = []

        # Iterate through all follower robots in the self.robots dictionary
        for name, bot in self.robots.items():
            if "follower" in name:
                # Get the arm joint positions (first 6 joints)
                arm_qpos = bot.core.joint_states.position[:6]
                qpos_list.append(arm_qpos)

                # Get the gripper joint position (7th joint) and normalize it
                gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(bot.core.joint_states.position[7])]
                qpos_list.append(gripper_qpos)

        # Concatenate all the positions into a single array
        return np.concatenate(qpos_list)


    def get_qvel(self):
        # Initialize a list to hold the arm and gripper velocities
        qvel_list = []

        # Iterate through all follower robots in the self.robots dictionary
        for name, bot in self.robots.items():
            if "follower" in name:
                # Get the arm joint velocities (first 6 joints)
                arm_qvel = bot.core.joint_states.velocity[:6]
                qvel_list.append(arm_qvel)

                # Get the gripper joint velocity (7th joint) and normalize it
                gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(bot.core.joint_states.velocity[6])]
                qvel_list.append(gripper_qvel)

        # Concatenate all the velocities into a single array
        return np.concatenate(qvel_list)

    def get_effort(self):
        # Initialize a list to hold the efforts for all arms and grippers
        effort_list = []

        # Iterate through all follower robots in the self.robots dictionary
        for name, bot in self.robots.items():
            if "follower" in name:
                # Get the first 7 effort values (6 for arm, 1 for gripper)
                robot_effort = bot.core.joint_states.effort[:7]
                effort_list.append(robot_effort)

        # Concatenate all the efforts into a single array
        return np.concatenate(effort_list)


    def get_images(self):
        return self.image_recorder.get_images()

    def get_base_vel(self):
        linear_vel = self.base.base.get_linear_velocity().x
        angular_vel = self.base.base.get_angular_velocity().z
        return np.array([linear_vel, angular_vel])

    def set_gripper_pose(self, robot_name, gripper_desired_pos_normalized):
        """
        Set the gripper position for a specific robot.

        Args:
            robot_name (str): The name of the robot (e.g., 'follower_left' or 'follower_right').
            gripper_desired_pos_normalized (float): The desired normalized gripper position.
        """
        # Unnormalize the gripper position
        desired_gripper_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(gripper_desired_pos_normalized)

        # Update the gripper command with the unnormalized position
        self.gripper_command.cmd = desired_gripper_joint

        # Publish the command to the corresponding robot's gripper
        self.robots[robot_name].gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]
        
        # Dynamically get all follower robots
        follower_robots = [robot for name, robot in self.robots.items() if 'follower' in name]
        
        # Move arms for all follower robots
        move_arms(
            follower_robots,
            [reset_position] * len(follower_robots),  # Repeat reset_position for each robot
            moving_time=1.0,
        )

    def _reset_gripper(self):
        """
        Set to position mode and do position resets.

        First open then close, then change back to PWM mode
        """
        # Dynamically get all follower robots
        follower_robots = [robot for name, robot in self.robots.items() if 'follower' in name]

        # Open the grippers for all follower robots
        move_grippers(
            follower_robots,
            [FOLLOWER_GRIPPER_JOINT_OPEN] * len(follower_robots),  # Set to open for all robots
            moving_time=0.5,
        )

        # Close the grippers for all follower robots
        move_grippers(
            follower_robots,
            [FOLLOWER_GRIPPER_JOINT_CLOSE] * len(follower_robots),  # Set to close for all robots
            moving_time=1.0,
        )


    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        if self.is_mobile:
            obs['base_vel'] = self.get_base_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            # Reboot gripper motors for all follower robots dynamically
            for robot_name, robot in self.robots.items():
                if 'follower' in robot_name:
                    robot.core.robot_reboot_motors('single', 'gripper', True)
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )
    
    def step(self, action, base_action=None, get_obs=True):
        follower_bots = {name: robot for name, robot in self.robots.items() if 'follower' in name}
        
        state_len = int(len(action) / len(follower_bots))  # Dynamically calculate per-bot state length
        index = 0

        # Iterate through each follower bot and set joint positions
        for name, robot in follower_bots.items():
            bot_action = action[index:index + state_len]
            robot.arm.set_joint_positions(bot_action[:6], blocking=False)  # Set arm positions
            self.set_gripper_pose(name, bot_action[-1])  # Set gripper position
            index += state_len


        if base_action is not None:
            base_action_linear, base_action_angular = base_action
            self.base.base.command_velocity_xyaw(x=base_action_linear, yaw=base_action_angular)

        # Optionally get observations
        obs = self.get_observation() if get_obs else None
        
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs
        )


def get_action(robots: dict):
    leader_bots = {name: robot for name, robot in robots.items() if 'leader' in name}
    
    num_leader_bots = len(leader_bots)
    action = np.zeros(num_leader_bots * 7)  # 6 joint positions + 1 gripper for each leader bot
    
    index = 0
    for robot_name, robot in leader_bots.items():
        # Arm actions (first 6 positions)
        action[index:index+6] = robot.core.joint_states.position[:6]
        # Gripper action (7th position)
        action[index+6] = LEADER_GRIPPER_JOINT_NORMALIZE_FN(robot.core.joint_states.position[6])
        index += 7

    return action


def make_real_env(
    node: InterbotixRobotNode = None,
    setup_robots: bool = True,
    setup_base: bool = False,
    torque_base: bool = False,
    config: dict = None
):
    if node is None:
        node = get_interbotix_global_node()
        if node is None:
            node = create_interbotix_global_node('aloha')
    env = RealEnv(
        node=node,
        setup_robots=setup_robots,
        setup_base=setup_base,
        torque_base=torque_base,
        config = config
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
