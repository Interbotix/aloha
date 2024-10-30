#!/usr/bin/env python3

from aloha.real_env import get_action, make_real_env
from aloha.robot_utils import (
    disable_gravity_compensation,
    enable_gravity_compensation,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    get_arm_gripper_positions,
    ImageRecorder,
    LEADER_GRIPPER_CLOSE_THRESH,
    LEADER_GRIPPER_JOINT_MID,
    load_yaml_file,
    move_arms,
    move_grippers,
    START_ARM_POSE,
    torque_off,
    torque_on,
)

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from typing import Dict
from tqdm import tqdm
import argparse
import cv2
import h5py
import numpy as np
import os
import rclpy
import time


def opening_ceremony(robots: Dict[str, InterbotixManipulatorXS], gravity_compensation: bool, dt: float) -> None:
    """
    Move all leader-follower pairs of robots to a starting pose for demonstration.

    :param robots: Dictionary of robots with robot names as keys.
    :param gravity_compensation: Boolean flag to enable gravity compensation for the leader robots.
    :param dt: Duration for each step in seconds.
    """
    # Separate leader and follower robots based on naming conventions
    leader_bots = {name: bot for name,
                   bot in robots.items() if "leader" in name}
    follower_bots = {name: bot for name,
                     bot in robots.items() if "follower" in name}

    # Initialize an empty list to store matched pairs of leader and follower robots
    pairs = []

    # Match leader and follower robots based on common suffixes (_left, _right, _solo)
    for leader_name, leader_bot in leader_bots.items():
        for suffix in ["_left", "_right", "_solo"]:
            if leader_name.endswith(suffix):
                follower_name = f"follower{suffix}"
                if follower_name in follower_bots:
                    pairs.append((leader_bot, follower_bots[follower_name]))
                break
        else:
            # Pair remaining leader and follower robots if no suffix match is found
            if follower_bots:
                _, follower_bot = follower_bots.popitem()
                pairs.append((leader_bot, follower_bot))

    # Check that at least one leader-follower pair is found
    if not pairs:
        raise ValueError(
            "No valid leader-follower pairs found in the robot dictionary.")

    # Initialize each pair by setting their operating modes and moving to start positions
    for leader_bot, follower_bot in pairs:
        # Reboot and configure follower's gripper motor
        follower_bot.core.robot_reboot_motors("single", "gripper", True)
        follower_bot.core.robot_set_operating_modes("group", "arm", "position")
        follower_bot.core.robot_set_operating_modes(
            "single", "gripper", "current_based_position")
        follower_bot.core.robot_set_motor_registers(
            "single", "gripper", "current_limit", 300)

        # Set leader robot's operating modes for arm and gripper
        leader_bot.core.robot_set_operating_modes("group", "arm", "position")
        leader_bot.core.robot_set_operating_modes(
            "single", "gripper", "position")

        # Enable torque for both leader and follower robots
        torque_on(follower_bot)
        torque_on(leader_bot)

        # Move both leader and follower robots to the starting arm position
        start_arm_qpos = START_ARM_POSE[:6]
        move_arms(
            bot_list=[leader_bot, follower_bot],
            target_pose_list=[start_arm_qpos] * 2,
            moving_time=4.0,
            DT=dt,
        )

        # Move both leader and follower grippers to the starting position
        move_grippers(
            [leader_bot, follower_bot],
            [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE],
            moving_time=0.5,
            DT=dt,
        )

    # Prepare to start data collection by disabling leader gripper torque and waiting for input
    for leader_bot in leader_bots.values():
        leader_bot.core.robot_torque_enable("single", "gripper", False)

    print("Close the grippers to start")

    # Wait for all leader grippers to close as an indication to start
    pressed = False
    while rclpy.ok() and not pressed:
        # Check if all leader grippers are closed
        pressed = all(
            get_arm_gripper_positions(leader_bot) < LEADER_GRIPPER_CLOSE_THRESH
            for leader_bot in leader_bots.values()
        )
        time.sleep(dt / 10)

    # Enable gravity compensation or turn off torque based on the parameter
    for leader_bot in leader_bots.values():
        if gravity_compensation:
            enable_gravity_compensation(leader_bot)
        else:
            torque_off(leader_bot)

    print("Started!")


def capture_one_episode(
    max_timesteps: int,
    dataset_dir: str,
    dataset_name: str,
    overwrite: bool,
    torque_base: bool = False,
    gravity_compensation: bool = False,
    config: Dict = None
) -> bool:
    """
    Capture one episode of robot teleoperation data and save it to a dataset file.

    :param max_timesteps: Maximum number of timesteps to capture in the episode.
    :param dataset_dir: Directory where the dataset will be saved.
    :param dataset_name: Name of the dataset file to create.
    :param overwrite: If True, overwrite existing dataset file if it exists.
    :param torque_base: Flag to enable base torque during recording.
    :param gravity_compensation: Enable gravity compensation on leader robots.
    :param config: Configuration dictionary containing robot and camera settings.
    :return: True if data collection is successful; False if data quality is low.
    """
    # Determine if the robot has a mobile base and set the control frequency
    IS_MOBILE = config.get("base", False)
    DT = 1 / config.get("fps", 50)

    # Initialize the ROS node and robot environment
    node = create_interbotix_global_node("aloha")
    env = make_real_env(
        node=node,
        setup_robots=False,
        setup_base=IS_MOBILE,
        torque_base=torque_base,
        config=config,
    )
    robot_startup(node)

    # Set up the dataset file path and handle overwrites
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    if os.path.isfile(dataset_path) and not overwrite:
        print(
            f"Dataset already exists at {dataset_path}\nHint: Set overwrite to True.")
        exit()

    # Move robots to starting position and wait for user to start
    opening_ceremony(
        env.robots, gravity_compensation=gravity_compensation, dt=DT)

    # Begin data collection
    ts = env.reset(fake=True)
    timesteps = [ts]
    actions = []
    actual_dt_history = []
    start_time = time.time()

    # Capture timesteps and actions in a loop
    for t in tqdm(range(max_timesteps)):
        t0 = time.time()
        action = get_action(env.robots)
        t1 = time.time()
        ts = env.step(action)
        t2 = time.time()
        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t0, t1, t2])
        time.sleep(max(0, DT - (time.time() - t0)))
    print(f"Avg fps: {max_timesteps / (time.time() - start_time)}")

    # End teleoperation and handle torque/gravity settings
    for name, robot in {name: bot for name, bot in env.robots.items() if "leader" in name}.items():
        if gravity_compensation:
            disable_gravity_compensation(robot)
        else:
            torque_on(robot)

    # Open grippers on follower robots
    follower_bots = {name: bot for name,
                     bot in env.robots.items() if "follower" in name}
    for name, bot in follower_bots.items():
        bot.core.robot_set_operating_modes("single", "gripper", "position")

    move_grippers(
        list(follower_bots.values()),
        [FOLLOWER_GRIPPER_JOINT_OPEN] * len(follower_bots),
        moving_time=0.5,
        DT=DT,
    )

    # Check the frequency of data collection for quality assurance
    freq_mean = print_dt_diagnosis(actual_dt_history)
    if freq_mean < 30:
        print(
            f"\n\nfreq_mean is {freq_mean}, lower than 30, re-collecting...\n\n\n\n")
        return False

    # Initialize dataset dictionary for storing observations and actions
    data_dict = {
        "/observations/qpos": [],
        "/observations/qvel": [],
        "/observations/effort": [],
        "/action": [],
    }
    if IS_MOBILE:
        data_dict["/base_action"] = []

    # Collect camera names from config and initialize image storage in data_dict
    camera_names = [camera["name"] for camera in config.get(
        "cameras", {}).get("camera_instances", [])]
    for cam_name in camera_names:
        data_dict[f"/observations/images/{cam_name}"] = []

    # Populate data_dict with recorded observations and actions
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        data_dict["/observations/qpos"].append(ts.observation["qpos"])
        data_dict["/observations/qvel"].append(ts.observation["qvel"])
        data_dict["/observations/effort"].append(ts.observation["effort"])
        data_dict["/action"].append(action)

        if IS_MOBILE:
            data_dict["/base_action"].append(ts.observation["base_vel"])

        for cam_name in camera_names:
            data_dict[f"/observations/images/{cam_name}"].append(
                ts.observation["images"][cam_name])

    # Optionally compress images and add padding for equal length
    COMPRESS = True
    if COMPRESS:
        t0 = time.time()
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        compressed_len = []
        for cam_name in camera_names:
            image_list = data_dict[f"/observations/images/{cam_name}"]
            compressed_list = []
            compressed_len.append([])
            for image in image_list:
                result, encoded_image = cv2.imencode(
                    ".jpg", image, encode_param)
                compressed_list.append(encoded_image)
                compressed_len[-1].append(len(encoded_image))
            data_dict[f"/observations/images/{cam_name}"] = compressed_list
        print(f"compression: {time.time() - t0:.2f}s")

        # Pad compressed images to ensure consistency in dataset size
        t0 = time.time()
        compressed_len = np.array(compressed_len)
        padded_size = compressed_len.max()
        for cam_name in camera_names:
            padded_images = []
            for compressed_image in data_dict[f"/observations/images/{cam_name}"]:
                padded_img = np.zeros(padded_size, dtype="uint8")
                padded_img[:len(compressed_image)] = compressed_image
                padded_images.append(padded_img)
            data_dict[f"/observations/images/{cam_name}"] = padded_images
        print(f"padding: {time.time() - t0:.2f}s")

    # Set the size for the datasets based on the number of follower robots
    total_size = 7 * len(follower_bots)

    # Write the data to an HDF5 file
    t0 = time.time()
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = False
        root.attrs["compress"] = COMPRESS
        obs = root.create_group("observations")
        image_group = obs.create_group("images")
        for cam_name in camera_names:
            shape = (max_timesteps, padded_size) if COMPRESS else (
                max_timesteps, 480, 640, 3)
            _ = image_group.create_dataset(
                cam_name, shape, dtype="uint8", chunks=(1, shape[1]))

        # Create datasets for joint positions, velocities, and efforts
        _ = obs.create_dataset("qpos", (max_timesteps, total_size))
        _ = obs.create_dataset("qvel", (max_timesteps, total_size))
        _ = obs.create_dataset("effort", (max_timesteps, total_size))
        _ = root.create_dataset("action", (max_timesteps, total_size))

        if IS_MOBILE:
            _ = root.create_dataset("base_action", (max_timesteps, 2))

        for name, array in data_dict.items():
            root[name][...] = array

        if COMPRESS:
            _ = root.create_dataset(
                "compress_len", (len(camera_names), max_timesteps))
            root["/compress_len"][...] = compressed_len

    print(f"Saving: {time.time() - t0:.1f} secs")

    robot_shutdown()
    return True


def check_episode_index(dataset_dir, episode_idx, data_suffix="hdf5"):
    """Checks if a file with the given episode index exists and prompts user for overwrite permission.

    Args:
        dataset_dir (str): Directory where episodes are stored.
        episode_idx (int): The episode index provided by the user.
        data_suffix (str): File extension for dataset files, defaults to 'hdf5'.

    Returns:
        bool: True if the file can be written (either doesn't exist or user agrees to overwrite).
              False if the user decides not to overwrite an existing file.
    """
    # Build the filename with the given episode index
    episode_file = os.path.join(
        dataset_dir, f"episode_{episode_idx}.{data_suffix}")

    if os.path.isfile(episode_file):
        # File exists; ask user for permission to overwrite
        user_input = (
            input(
                f"Episode file '{episode_file}' already exists. Do you want to overwrite it? (y/n): "
            )
            .strip()
            .lower()
        )
        if user_input == "y":
            print(f"Overwriting episode {episode_idx}.")
            return True
        else:
            print("Not overwriting the file. Operation aborted.")
            return False
    else:
        return True


def get_auto_index(dataset_dir, dataset_name_prefix="", data_suffix="hdf5"):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx + 1):
        if not os.path.isfile(
            os.path.join(
                dataset_dir, f"{dataset_name_prefix}episode_{i}.{data_suffix}")
        ):
            return i
    raise Exception(
        f"Error getting auto index, or more than {max_idx} episodes")


def print_dt_diagnosis(actual_dt_history):
    actual_dt_history = np.array(actual_dt_history)
    get_action_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
    step_env_time = actual_dt_history[:, 2] - actual_dt_history[:, 1]
    total_time = actual_dt_history[:, 2] - actual_dt_history[:, 0]

    dt_mean = np.mean(total_time)
    # dt_std = np.std(total_time)
    freq_mean = 1 / dt_mean
    print(
        (
            f"Avg freq: {freq_mean:.2f} Get action: {np.mean(get_action_time):.3f} "
            f"Step env: {np.mean(step_env_time):.3f}"
        )
    )
    return freq_mean


def debug():
    print("====== Debug mode ======")
    image_recorder = ImageRecorder(init_node=False, is_debug=True)
    while True:
        time.sleep(1)
        image_recorder.print_diagnostics()


def main(args: dict):

    torque_base = args.get("enable_base_torque", False)
    gravity_compensation = args.get("gravity_compensation", False)

    robot_base = args.get("robot", "")

    config = load_yaml_file("robot", robot_base).get('robot', {})

    task_config = load_yaml_file("task")
    task = task_config["tasks"].get(args.get("task_name"))
    dataset_dir = os.path.expanduser(task.get("dataset_dir"))
    max_timesteps = task.get("episode_len")

    if args["episode_idx"] is not None:
        episode_idx = args["episode_idx"]
    else:
        episode_idx = get_auto_index(dataset_dir)

    overwrite = check_episode_index(
        dataset_dir=dataset_dir, episode_idx=episode_idx)

    if not overwrite:
        exit()

    dataset_name = f"episode_{episode_idx}"
    print(dataset_name + "\n")
    while True:
        is_healthy = capture_one_episode(
            max_timesteps,
            dataset_dir,
            dataset_name,
            overwrite,
            torque_base,
            gravity_compensation,
            config,
        )
        if is_healthy:
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--task_name",
        action="store",
        type=str,
        help="Task name.",
        required=True,
    )
    parser.add_argument(
        "--episode_idx",
        action="store",
        type=int,
        help="Episode index.",
        default=None,
        required=False,
    )
    parser.add_argument(
        "-b",
        "--enable_base_torque",
        action="store_true",
        help=(
            "If set, mobile base will be torqued on during episode recording, allowing the use of"
            " a joystick controller or some other manual method."
        ),
    )
    parser.add_argument(
        "-g",
        "--gravity_compensation",
        action="store_true",
        help="If set, gravity compensation will be enabled for the leader robots when teleop starts.",
    )
    parser.add_argument(
        "-r",
        "--robot",
        action="store",
        type=str,
        help="Robot Setup.",
        required=True,
    )
    main(vars(parser.parse_args()))
