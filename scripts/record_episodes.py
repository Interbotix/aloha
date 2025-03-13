#!/usr/bin/env python3

import argparse
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
import cv2
import h5py
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import os
from pathlib import Path
import rclpy
import time
from typing import Dict, List
from tqdm import tqdm


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

    # Create dictionaries mapping suffixes to leader and follower robots
    leader_suffixes = {name.split('_', 1)[1]: bot for name, bot in leader_bots.items()}
    follower_suffixes = {name.split('_', 1)[1]: bot for name, bot in follower_bots.items()}

    # Pair leader and follower robots based on matching suffixes
    for suffix, leader_bot in leader_suffixes.items():
        if suffix in follower_suffixes:
            # If matching follower exists, pair it with the leader
            follower_bot = follower_suffixes.pop(suffix)
            pairs.append((leader_bot, follower_bot))
        else:
            # Raise an error if there’s an unmatched leader suffix
            raise ValueError(f"Unmatched leader suffix '{suffix}' found. Every leader should have a corresponding follower with the same suffix.")

    # Check if any unmatched followers remain after pairing
    if follower_suffixes:
        unmatched_suffixes = ', '.join(follower_suffixes.keys())
        raise ValueError(f"Unmatched follower suffix(es) found: {unmatched_suffixes}. Every follower should have a corresponding leader with the same suffix.")

    # Ensure at least one leader-follower pair was created
    if not pairs:
        raise ValueError("No valid leader-follower pairs found in the robot dictionary.")


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
            dt=dt,
        )

        # Move both leader and follower grippers to the starting position
        move_grippers(
            [leader_bot, follower_bot],
            [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE],
            moving_time=0.5,
            dt=dt,
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
        dt=DT,
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
    
    if camera_names:
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
    if COMPRESS and camera_names:
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
        if camera_names:
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

        if COMPRESS and camera_names:
            _ = root.create_dataset(
                "compress_len", (len(camera_names), max_timesteps))
            root["/compress_len"][...] = compressed_len

    print(f"Saving: {time.time() - t0:.1f} secs")

    robot_shutdown()
    return True


def check_episode_index(dataset_dir: str, episode_idx: int, data_suffix: str = "hdf5") -> bool:
    """
    Checks if a file with the specified episode index exists, and prompts the user for overwrite 
    permission if the file is present.

    :param dataset_dir: Directory where episodes are stored.
    :param episode_idx: The episode index provided by the user.
    :param data_suffix: File extension for dataset files, defaults to 'hdf5'.
    :return: True if the file can be written (either does not exist or user agrees to overwrite); 
             False if the user decides not to overwrite an existing file.
    """
    # Construct the full file path for the episode
    episode_file = os.path.join(
        dataset_dir, f"episode_{episode_idx}.{data_suffix}")

    # Check if the file exists
    if os.path.isfile(episode_file):
        # Prompt user for overwrite permission if file exists
        user_input = input(
            f"Episode file '{episode_file}' already exists. Do you want to overwrite it? (y/n): "
        ).strip().lower()

        if user_input == "y":
            print(f"Overwriting episode {episode_idx}.")
            return True
        else:
            print("Not overwriting the file. Operation aborted.")
            return False
    return True


def get_auto_index(dataset_dir: str, dataset_name_prefix: str = "", data_suffix: str = "hdf5") -> int:
    """
    Determines the next available episode index in a dataset directory. Creates the directory if it 
    does not exist. Searches for the first unused index from 0 to `max_idx`.

    :param dataset_dir: Directory where dataset episodes are stored.
    :param dataset_name_prefix: Optional prefix for episode file names.
    :param data_suffix: File extension for dataset files, defaults to 'hdf5'.
    :return: The next available index for a new episode file.
    :raises Exception: If the index limit is reached or another error occurs.
    """
    max_idx = 1000

    # Ensure the dataset directory exists
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)

    # Iterate through indices to find the first available one
    for i in range(max_idx + 1):
        episode_file = os.path.join(
            dataset_dir, f"{dataset_name_prefix}episode_{i}.{data_suffix}")
        if not os.path.isfile(episode_file):
            return i

    # Raise an exception if no available index is found within the range
    raise Exception(
        f"Error getting auto index, or more than {max_idx} episodes.")


def print_dt_diagnosis(actual_dt_history: List[List[float]]) -> float:
    """
    Diagnoses and prints timing statistics for each step in the episode, such as the frequency of 
    action execution and environment steps.

    :param actual_dt_history: A list of timestamp records for each timestep. Each inner list 
                              contains three floats representing:
                                - Start time of getting action
                                - End time of getting action/start of step environment
                                - End time of step environment
    :return: The mean frequency of the total time taken for each step.
    """
    actual_dt_history = np.array(actual_dt_history)
    get_action_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
    step_env_time = actual_dt_history[:, 2] - actual_dt_history[:, 1]
    total_time = actual_dt_history[:, 2] - actual_dt_history[:, 0]

    dt_mean = np.mean(total_time)
    freq_mean = 1 / dt_mean

    print(
        f"Avg freq: {freq_mean:.2f} Get action: {np.mean(get_action_time):.3f} "
        f"Step env: {np.mean(step_env_time):.3f}"
    )

    return freq_mean


def debug() -> None:
    """
    Runs the program in debug mode, creating an `ImageRecorder` instance to record diagnostics.
    Periodically prints diagnostic information such as image frequency and quality for debugging purposes.
    """
    print("====== Debug mode ======")

    # Initialize ImageRecorder in debug mode without initializing ROS node
    image_recorder = ImageRecorder(init_node=False, is_debug=True)

    while True:
        time.sleep(1)
        image_recorder.print_diagnostics()


def main(args: Dict[str, any]) -> None:
    """
    Main function for executing a robot teleoperation task based on configuration parameters.
    Handles dataset setup, task configuration, and runs the teleoperation loop to capture episodes.

    :param args: Dictionary of arguments, expected keys:
        - "enable_base_torque" (bool): Whether to enable torque on the base (for mobile robots).
        - "gravity_compensation" (bool): Whether to enable gravity compensation for leader robots.
        - "robot" (str): Robot setup configuration (e.g., 'aloha_solo', 'aloha_stationary', 'aloha_mobile').
        - "task_name" (str): Task name used to fetch specific task configuration.
        - "episode_idx" (Optional[int]): Episode index for dataset naming; if None, auto-indexing is used.
    """
    # Retrieve arguments and configuration settings
    torque_base = args.get("enable_base_torque", False)
    gravity_compensation = args.get("gravity_compensation", False)
    robot_base = args.get("robot", "")

    base_path = Path(__file__).resolve().parent.parent / "config"


    # Load robot and task configurations from YAML files
    config = load_yaml_file("robot", robot_base, base_path).get('robot', {})
    task_config = load_yaml_file("task", base_path=base_path)
    task = task_config["tasks"].get(args.get("task_name"))

    # Determine dataset directory and maximum timesteps
    dataset_dir = os.path.expanduser(task.get("dataset_dir"))
    max_timesteps = task.get("episode_len")

    # Determine episode index (auto-index if not provided)
    if args["episode_idx"] is not None:
        episode_idx = args["episode_idx"]
    else:
        episode_idx = get_auto_index(dataset_dir)

    # Check for overwrite permission if dataset already exists
    overwrite = check_episode_index(
        dataset_dir=dataset_dir, episode_idx=episode_idx)
    if not overwrite:
        exit()

    # Generate dataset name based on episode index
    dataset_name = f"episode_{episode_idx}"
    print(f"{dataset_name}\n")

    # Start capturing an episode in a loop until it completes successfully
    while True:
        is_healthy = capture_one_episode(
            max_timesteps=max_timesteps,
            dataset_dir=dataset_dir,
            dataset_name=dataset_name,
            overwrite=overwrite,
            torque_base=torque_base,
            gravity_compensation=gravity_compensation,
            config=config,
        )
        if is_healthy:
            break


if __name__ == "__main__":
    # Argument parser to manage command-line inputs
    parser = argparse.ArgumentParser(
        description="Launches robot teleoperation with specified parameters.")

    # Task-specific argument: required
    parser.add_argument(
        "-t",
        "--task_name",
        action="store",
        type=str,
        help="Task name to specify the teleoperation task.",
        required=True,
    )

    # Episode index argument: optional, defaults to auto-indexing if not provided
    parser.add_argument(
        "--episode_idx",
        action="store",
        type=int,
        help="Episode index to name the dataset file. Auto-generated if not provided.",
        default=None,
        required=False,
    )

    # Base torque enabling flag: optional
    parser.add_argument(
        "-b",
        "--enable_base_torque",
        action="store_true",
        help=(
            "Enable base torque for mobile robots during recording. Allows joystick control or"
            " other manual methods."
        ),
    )

    # Gravity compensation enabling flag: optional
    parser.add_argument(
        "-g",
        "--gravity_compensation",
        action="store_true",
        help="Enable gravity compensation for leader robots at the start of teleoperation.",
    )

    # Robot setup configuration: required
    parser.add_argument(
        "-r",
        "--robot",
        action="store",
        type=str,
        help="Robot setup configuration (e.g., aloha_solo, aloha_stationary, aloha_mobile).",
        required=True,
    )

    # Execute the main function with parsed arguments
    main(vars(parser.parse_args()))
