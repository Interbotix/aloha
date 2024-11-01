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

    def __init__(
        self,
        node: InterbotixRobotNode,
        setup_robots: bool = True,
        setup_base: bool = False,
        torque_base: bool = False,
        config: dict = None,
    ):
        """
        Initialize the Real Robot Environment.

        :param node: The InterbotixRobotNode instance used for configuring and controlling the robot.
        :param setup_robots: If True, sets up the robot arms during initialization, defaults to True.
        :param setup_base: If True, configures the robot's base, defaults to False.
        :param torque_base: If True, enables torque on the robot's base after setup. Only relevant if
                            the base is mobile, defaults to False.
        :param config: Dictionary of configuration parameters, including base type and robot settings.

        :raises ValueError: Raised if setup_base is set to False but the robot base is expected to be mobile.
        """
        self.is_mobile = config.get('base', False)

        self.dt = 1 / config.get('fps', 30)

        # Dynamically import module based on config value
        if self.is_mobile:
            xs_robot_module = importlib.import_module(
                'interbotix_xs_modules.xs_robot.slate')
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

        # Raise an error if no robots were added to the dictionary
        if not self.robots:
            raise ValueError(
                "No robots were initialized. Check YAML configuration for 'leader_arms' and 'follower_arms'.")

        self.follower_bots = [robot for name,
                              robot in self.robots.items() if 'follower' in name]

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
                    "Hint: Update the robot config file to enable the base."
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
                # Get the arm joint positions
                arm_qpos = bot.arm.get_joint_positions()
                qpos_list.append(arm_qpos)

                # Get the gripper joint position and normalize it
                gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(
                    bot.gripper.get_gripper_position())]
                qpos_list.append(gripper_qpos)

        # Concatenate all the positions into a single array
        return np.concatenate(qpos_list)

    def get_qvel(self):
        # Initialize a list to hold the arm and gripper velocities
        qvel_list = []

        # Iterate through all follower robots in the self.robots dictionary
        for name, bot in self.robots.items():
            if "follower" in name:
                # Get the arm joint velocities
                arm_qvel = bot.arm.get_joint_velocities()
                qvel_list.append(arm_qvel)

                # Get the gripper joint velocity and normalize it
                gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(
                    bot.gripper.get_gripper_velocity())]
                qvel_list.append(gripper_qvel)

        # Concatenate all the velocities into a single array
        return np.concatenate(qvel_list)

    def get_effort(self):
        """
        Gather and concatenate efforts for all follower robots' arms and grippers.

        Returns:
            np.ndarray: Array of concatenated efforts for arms and grippers.
        """
        # Initialize a list to hold the efforts for all arms and grippers
        effort_list = []

        # Iterate through all follower robots in the self.robots dictionary
        for name, bot in self.robots.items():
            if "follower" in name:
                # Get the effort values for arm and gripper, wrapping gripper effort in a list
                arm_effort = bot.arm.get_joint_efforts()            # Array of arm joint efforts
                gripper_effort = [bot.gripper.get_gripper_effort()] # Wrap single float in list
                # Append both arm and gripper efforts to the effort_list
                effort_list.append(arm_effort)
                effort_list.append(gripper_effort)

        # Concatenate all efforts into a single array
        return np.concatenate(effort_list)

    def get_images(self):
        return self.image_recorder.get_images()

    def get_base_vel(self):
        linear_vel = self.base.base.get_linear_velocity().x
        angular_vel = self.base.base.get_angular_velocity().z
        return np.array([linear_vel, angular_vel])

    def set_gripper_pose(self, robot_name: str, gripper_desired_pos_normalized: float) -> None:
        """
        Set the gripper position for a specific robot.

        :param robot_name: The name of the robot (e.g., 'follower_left' or 'follower_right').
        :param gripper_desired_pos_normalized: The desired gripper position, normalized as a float value.
        :return: None
        """
        # Unnormalize the gripper position
        desired_gripper_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(
            gripper_desired_pos_normalized)

        # Update the gripper command with the unnormalized position
        self.gripper_command.cmd = desired_gripper_joint

        # Publish the command to the corresponding robot's gripper
        self.robots[robot_name].gripper.core.pub_single.publish(
            self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]

        # Move arms for all follower robots
        move_arms(
            bot_list=self.follower_bots,
            # Repeat reset_position for each robot
            target_pose_list=[reset_position] * len(self.follower_bots),
            dt=self.dt,
        )

    def _reset_gripper(self):
        """
        Set to position mode and do position resets.

        First open then close, then change back to PWM mode
        """

        # Open the grippers for all follower robots
        move_grippers(
            self.follower_bots,
            [FOLLOWER_GRIPPER_JOINT_OPEN] *
            len(self.follower_bots),  # Set to open for all robots
            moving_time=0.5,
            dt=self.dt,
        )

        # Close the grippers for all follower robots
        move_grippers(
            self.follower_bots,
            [FOLLOWER_GRIPPER_JOINT_CLOSE] *
            len(self.follower_bots),  # Set to close for all robots
            moving_time=1.0,
            dt=self.dt,
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

        follower_robots = {name: robot for name,
                           robot in self.robots.items() if 'follower' in name}

        # Dynamically calculate per-bot state length
        state_len = int(len(action) / len(follower_robots))
        index = 0

        # Iterate through each follower bot and set joint positions
        for name, robot in follower_robots.items():
            bot_action = action[index:index + state_len]
            robot.arm.set_joint_positions(
                bot_action[:-1], blocking=False)  # Set arm positions
            self.set_gripper_pose(name, bot_action[-1])  # Set gripper position
            index += state_len

        if base_action is not None:
            base_action_linear, base_action_angular = base_action
            self.base.base.command_velocity_xyaw(
                x=base_action_linear, yaw=base_action_angular)

        # Optionally get observations
        obs = self.get_observation() if get_obs else None

        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs,
        )


def get_action(robots: dict[str, InterbotixManipulatorXS]):
    leader_bots = {name: robot for name,
                   robot in robots.items() if 'leader' in name}

    # Dynamically determine the number of joints based on the first leader bot
    num_arm_joints = next(iter(leader_bots.values())).arm.group_info.num_joints
    total_joints_per_robot = num_arm_joints + 1  # +1 for the gripper position

    # Initialize the action array for all leader bots
    action = np.zeros(len(leader_bots) * total_joints_per_robot)

    index = 0
    for robot in leader_bots.values():
        # Arm actions
        action[index:index+num_arm_joints] = robot.arm.get_joint_positions()
        # Gripper action
        action[index+num_arm_joints] = LEADER_GRIPPER_JOINT_NORMALIZE_FN(
            robot.gripper.get_gripper_position())
        index += total_joints_per_robot

    return action


def make_real_env(
    node: InterbotixRobotNode = None,
    setup_robots: bool = True,
    setup_base: bool = False,
    torque_base: bool = False,
    config: dict = None,
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
        config=config,
    )
    return env


def test_real_teleop():
    """
    Test bimanual teleoperation and show image observations onscreen.

    It first reads joint poses from both leader arms.
    Then use it as actions to step the environment.
    The environment returns full observations including images.
    config: dict = None,
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
            plt.pause(env.dt)
        else:
            time.sleep(env.dt)


if __name__ == '__main__':
    test_real_teleop()
