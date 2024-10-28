from collections import deque
import time
from typing import Sequence
import os

from cv_bridge import CvBridge
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gravity_compensation import (
    InterbotixGravityCompensationInterface,
)
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

class ImageRecorder:
    def __init__(
        self,
        config: dict,
        is_debug: bool = False,
        node: Node = None,
    ):
        self.is_debug = is_debug
        self.bridge = CvBridge()

        # Get camera names from config dictionary
        self.camera_names = [camera['name'] for camera in config.get('cameras', {}).get('camera_instances', [])]


        # Dynamically create attributes and subscriptions for each camera
        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)

            # Create appropriate callback dynamically
            callback_func = self.create_callback(cam_name)

            # Subscribe to the camera topic
            topic = COLOR_IMAGE_TOPIC_NAME.format(cam_name)
            node.create_subscription(Image, topic, callback_func, 20)

            # If in debug mode, create a deque to store timestamps
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))

        time.sleep(0.5)

    def create_callback(self, cam_name: str):
        """Creates a callback function dynamically for a given camera name."""
        def callback(data: Image):
            self.image_cb(cam_name, data)
        return callback

    def image_cb(self, cam_name: str, data: Image):
        """Handles the incoming image data for the specified camera."""
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

    def get_images(self):
        """Returns a dictionary of the latest images from all cameras."""
        image_dict = {}
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    def print_diagnostics(self):
        """Prints diagnostic information such as image frequency for each camera."""
        def dt_helper(ts):
            ts = np.array(ts)
            diff = ts[1:] - ts[:-1]
            return np.mean(diff)
        
        for cam_name in self.camera_names:
            timestamps = getattr(self, f'{cam_name}_timestamps', [])
            if timestamps:
                image_freq = 1 / dt_helper(timestamps)
                print(f'{cam_name} {image_freq=:.2f}')
        print()



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


def enable_gravity_compensation(bot: InterbotixManipulatorXS):
    gravity_compensation = InterbotixGravityCompensationInterface(bot.core)
    gravity_compensation.enable()


def disable_gravity_compensation(bot: InterbotixManipulatorXS):
    gravity_compensation = InterbotixGravityCompensationInterface(bot.core)
    gravity_compensation.disable()


JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
START_ARM_POSE = [
    0.0, -0.96, 1.16, 0.0, -0.3, 0.0, 0.02239, -0.02239,
    0.0, -0.96, 1.16, 0.0, -0.3, 0.0, 0.02239, -0.02239,
]

LEADER_GRIPPER_CLOSE_THRESH = 0.0

# Left finger position limits (qpos[7]), right_finger = -1 * left_finger
LEADER_GRIPPER_POSITION_OPEN = 0.0323
LEADER_GRIPPER_POSITION_CLOSE = 0.0185

FOLLOWER_GRIPPER_POSITION_OPEN = 0.0579
FOLLOWER_GRIPPER_POSITION_CLOSE = 0.0440

# Gripper joint limits (qpos[6])
LEADER_GRIPPER_JOINT_OPEN = 0.8298
LEADER_GRIPPER_JOINT_CLOSE = -0.0552

FOLLOWER_GRIPPER_JOINT_OPEN = 1.6214
FOLLOWER_GRIPPER_JOINT_CLOSE = 0.6197

### Helper functions

LEADER_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - LEADER_GRIPPER_POSITION_CLOSE) / (LEADER_GRIPPER_POSITION_OPEN - LEADER_GRIPPER_POSITION_CLOSE)
FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - FOLLOWER_GRIPPER_POSITION_CLOSE) / (FOLLOWER_GRIPPER_POSITION_OPEN - FOLLOWER_GRIPPER_POSITION_CLOSE)
LEADER_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (LEADER_GRIPPER_POSITION_OPEN - LEADER_GRIPPER_POSITION_CLOSE) + LEADER_GRIPPER_POSITION_CLOSE
FOLLOWER_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (FOLLOWER_GRIPPER_POSITION_OPEN - FOLLOWER_GRIPPER_POSITION_CLOSE) + FOLLOWER_GRIPPER_POSITION_CLOSE
LEADER2FOLLOWER_POSITION_FN = lambda x: FOLLOWER_GRIPPER_POSITION_UNNORMALIZE_FN(LEADER_GRIPPER_POSITION_NORMALIZE_FN(x))

LEADER_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - LEADER_GRIPPER_JOINT_CLOSE) / (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE)
FOLLOWER_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - FOLLOWER_GRIPPER_JOINT_CLOSE) / (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE)
LEADER_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE) + LEADER_GRIPPER_JOINT_CLOSE
FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE) + FOLLOWER_GRIPPER_JOINT_CLOSE
LEADER2FOLLOWER_JOINT_FN = lambda x: FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(LEADER_GRIPPER_JOINT_NORMALIZE_FN(x))

LEADER_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (LEADER_GRIPPER_POSITION_OPEN - LEADER_GRIPPER_POSITION_CLOSE)
FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (FOLLOWER_GRIPPER_POSITION_OPEN - FOLLOWER_GRIPPER_POSITION_CLOSE)

LEADER_POS2JOINT = lambda x: LEADER_GRIPPER_POSITION_NORMALIZE_FN(x) * (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE) + LEADER_GRIPPER_JOINT_CLOSE
LEADER_JOINT2POS = lambda x: LEADER_GRIPPER_POSITION_UNNORMALIZE_FN((x - LEADER_GRIPPER_JOINT_CLOSE) / (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE))
FOLLOWER_POS2JOINT = lambda x: FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(x) * (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE) + FOLLOWER_GRIPPER_JOINT_CLOSE
FOLLOWER_JOINT2POS = lambda x: FOLLOWER_GRIPPER_POSITION_UNNORMALIZE_FN((x - FOLLOWER_GRIPPER_JOINT_CLOSE) / (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE))

LEADER_GRIPPER_JOINT_MID = (LEADER_GRIPPER_JOINT_OPEN + LEADER_GRIPPER_JOINT_CLOSE)/2


# RealSense cameras image topic (realsense2_camera v4.55 and up)
COLOR_IMAGE_TOPIC_NAME = '{}/camera/color/image_rect_raw' # Change to robot.yaml


### ALOHA Fixed Constants
DT = 0.02

try:
    from rclpy.duration import Duration
    from rclpy.constants import S_TO_NS
    DT_DURATION = Duration(seconds=0, nanoseconds=DT * S_TO_NS)
except ImportError:
    pass

FPS = 50
