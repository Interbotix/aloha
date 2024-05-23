from collections import deque
import time
from typing import Sequence

from aloha.constants import (
    COLOR_IMAGE_TOPIC_NAME,
    DT,
    IS_MOBILE,
)
from cv_bridge import CvBridge
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState


class ImageRecorder:
    def __init__(
        self,
        is_mobile: bool = IS_MOBILE,
        is_debug: bool = False,
        node: Node = None,
    ):
        self.is_debug = is_debug
        self.bridge = CvBridge()

        if is_mobile:
            self.camera_names = ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
        else:
            self.camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']

        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
            elif cam_name == 'cam_low':
                callback_func = self.image_cb_cam_low
            elif cam_name == 'cam_left_wrist':
                callback_func = self.image_cb_cam_left_wrist
            elif cam_name == 'cam_right_wrist':
                callback_func = self.image_cb_cam_right_wrist
            else:
                raise NotImplementedError
            topic = COLOR_IMAGE_TOPIC_NAME.format(cam_name)
            node.create_subscription(Image, topic, callback_func, 20)
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))
        time.sleep(0.5)

    def image_cb(self, cam_name: str, data: Image):
        setattr(
            self,
            f'{cam_name}_image',
            self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        )
        setattr(self, f'{cam_name}_secs', data.header.stamp.sec)
        setattr(self, f'{cam_name}_nsecs', data.header.stamp.nanosec)
        if self.is_debug:
            getattr(
                self,
                f'{cam_name}_timestamps'
            ).append(data.header.stamp.sec + data.header.stamp.sec * 1e-9)

    def image_cb_cam_high(self, data):
        cam_name = 'cam_high'
        return self.image_cb(cam_name, data)

    def image_cb_cam_low(self, data):
        cam_name = 'cam_low'
        return self.image_cb(cam_name, data)

    def image_cb_cam_left_wrist(self, data):
        cam_name = 'cam_left_wrist'
        return self.image_cb(cam_name, data)

    def image_cb_cam_right_wrist(self, data):
        cam_name = 'cam_right_wrist'
        return self.image_cb(cam_name, data)

    def get_images(self):
        image_dict = {}
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    def print_diagnostics(self):
        def dt_helper(ts):
            ts = np.array(ts)
            diff = ts[1:] - ts[:-1]
            return np.mean(diff)
        for cam_name in self.camera_names:
            image_freq = 1 / dt_helper(getattr(self, f'{cam_name}_timestamps'))
            print(f'{cam_name} {image_freq=:.2f}')
        print()


class Recorder:
    def __init__(
        self,
        side: str,
        is_debug: bool = False,
        node: Node = None,
    ):
        self.secs = None
        self.nsecs = None
        self.qpos = None
        self.effort = None
        self.arm_command = None
        self.gripper_command = None
        self.is_debug = is_debug

        node.create_subscription(
            JointState,
            f'/follower_{side}/joint_states',
            self.follower_state_cb,
            10,
        )
        node.create_subscription(
            JointGroupCommand,
            f'/follower_{side}/commands/joint_group',
            self.follower_arm_commands_cb,
            10,
        )
        node.create_subscription(
            JointSingleCommand,
            f'/follower_{side}/commands/joint_single',
            self.follower_gripper_commands_cb,
            10,
        )
        if self.is_debug:
            self.joint_timestamps = deque(maxlen=50)
            self.arm_command_timestamps = deque(maxlen=50)
            self.gripper_command_timestamps = deque(maxlen=50)
        time.sleep(0.1)

    def follower_state_cb(self, data: JointState):
        self.qpos = data.position
        self.qvel = data.velocity
        self.effort = data.effort
        self.data = data
        if self.is_debug:
            self.joint_timestamps.append(time.time())

    def follower_arm_commands_cb(self, data: JointGroupCommand):
        self.arm_command = data.cmd
        if self.is_debug:
            self.arm_command_timestamps.append(time.time())

    def follower_gripper_commands_cb(self, data: JointSingleCommand):
        self.gripper_command = data.cmd
        if self.is_debug:
            self.gripper_command_timestamps.append(time.time())

    def print_diagnostics(self):
        def dt_helper(ts):
            ts = np.array(ts)
            diff = ts[1:] - ts[:-1]
            return np.mean(diff)

        joint_freq = 1 / dt_helper(self.joint_timestamps)
        arm_command_freq = 1 / dt_helper(self.arm_command_timestamps)
        gripper_command_freq = 1 / dt_helper(self.gripper_command_timestamps)

        print(f'{joint_freq=:.2f}\n{arm_command_freq=:.2f}\n{gripper_command_freq=:.2f}\n')


def get_arm_joint_positions(bot: InterbotixManipulatorXS):
    return bot.arm.core.joint_states.position[:6]


def get_arm_gripper_positions(bot: InterbotixManipulatorXS):
    joint_position = bot.gripper.core.joint_states.position[6]
    return joint_position


def move_arms(
    bot_list: Sequence[InterbotixManipulatorXS],
    target_pose_list: Sequence[Sequence[float]],
    moving_time: float = 1.0,
) -> None:
    num_steps = int(moving_time / DT)
    curr_pose_list = [get_arm_joint_positions(bot) for bot in bot_list]
    zipped_lists = zip(curr_pose_list, target_pose_list)
    traj_list = [
        np.linspace(curr_pose, target_pose, num_steps) for curr_pose, target_pose in zipped_lists
    ]
    for t in range(num_steps):
        for bot_id, bot in enumerate(bot_list):
            bot.arm.set_joint_positions(traj_list[bot_id][t], blocking=False)
        time.sleep(DT)


def sleep_arms(
    bot_list: Sequence[InterbotixManipulatorXS],
    moving_time: float = 5.0,
    home_first: bool = True,
) -> None:
    """Command given list of arms to their sleep poses, optionally to their home poses first.

    :param bot_list: List of bots to command to their sleep poses
    :param moving_time: Duration in seconds the movements should take, defaults to 5.0
    :param home_first: True to command the arms to their home poses first, defaults to True
    """
    if home_first:
        move_arms(
            bot_list,
            [[0.0, -0.96, 1.16, 0.0, -0.3, 0.0]] * len(bot_list),
            moving_time=moving_time
        )
    move_arms(
        bot_list,
        [bot.arm.group_info.joint_sleep_positions for bot in bot_list],
        moving_time=moving_time,
    )

def move_grippers(
    bot_list: Sequence[InterbotixManipulatorXS],
    target_pose_list: Sequence[float],
    moving_time: float,
):
    gripper_command = JointSingleCommand(name='gripper')
    num_steps = int(moving_time / DT)
    curr_pose_list = [get_arm_gripper_positions(bot) for bot in bot_list]
    zipped_lists = zip(curr_pose_list, target_pose_list)
    traj_list = [
        np.linspace(curr_pose, target_pose, num_steps) for curr_pose, target_pose in zipped_lists
    ]
    for t in range(num_steps):
        for bot_id, bot in enumerate(bot_list):
            gripper_command.cmd = traj_list[bot_id][t]
            bot.gripper.core.pub_single.publish(gripper_command)
        time.sleep(DT)


def setup_follower_bot(bot: InterbotixManipulatorXS):
    bot.core.robot_reboot_motors('single', 'gripper', True)
    bot.core.robot_set_operating_modes('group', 'arm', 'position')
    bot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
    torque_on(bot)


def setup_leader_bot(bot: InterbotixManipulatorXS):
    bot.core.robot_set_operating_modes('group', 'arm', 'pwm')
    bot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
    torque_off(bot)


def set_standard_pid_gains(bot: InterbotixManipulatorXS):
    bot.core.robot_set_motor_registers('group', 'arm', 'Position_P_Gain', 800)
    bot.core.robot_set_motor_registers('group', 'arm', 'Position_I_Gain', 0)


def set_low_pid_gains(bot: InterbotixManipulatorXS):
    bot.core.robot_set_motor_registers('group', 'arm', 'Position_P_Gain', 100)
    bot.core.robot_set_motor_registers('group', 'arm', 'Position_I_Gain', 0)


def torque_off(bot: InterbotixManipulatorXS):
    bot.core.robot_torque_enable('group', 'arm', False)
    bot.core.robot_torque_enable('single', 'gripper', False)


def torque_on(bot: InterbotixManipulatorXS):
    bot.core.robot_torque_enable('group', 'arm', True)
    bot.core.robot_torque_enable('single', 'gripper', True)


def calibrate_linear_vel(base_action, c=None):
    if c is None:
        c = 0.
    v = base_action[..., 0]
    w = base_action[..., 1]
    base_action = base_action.copy()
    base_action[..., 0] = v - c * w
    return base_action


def smooth_base_action(base_action):
    return np.stack(
        [
            np.convolve(
                base_action[:, i],
                np.ones(5)/5, mode='same') for i in range(base_action.shape[1])
        ],
        axis=-1
    ).astype(np.float32)


def postprocess_base_action(base_action):
    linear_vel, angular_vel = base_action
    angular_vel *= 0.9
    return np.array([linear_vel, angular_vel])
