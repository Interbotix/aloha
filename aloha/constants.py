# flake8: noqa

import os
from rclpy.duration import Duration
from rclpy.constants import S_TO_NS

### Task parameters

### Set to 'true for Mobile ALOHA, 'false' for Stationary ALOHA
IS_MOBILE = os.environ.get('INTERBOTIX_ALOHA_IS_MOBILE', 'true').lower() == 'true'

COLOR_IMAGE_TOPIC_NAME = '{}/color/image_rect_raw'  # for RealSense cameras
# COLOR_IMAGE_TOPIC_NAME = 'usb_{}/image_raw'  # for USB cameras

DATA_DIR = os.path.expanduser('~/data')

### ALOHA fixed constants
DT = 0.02
DT_DURATION = Duration(seconds=0, nanoseconds=DT * S_TO_NS)

FPS = 50
JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
START_ARM_POSE = [
    0.0, -0.96, 1.16, 0.0, -0.3, 0.0, 0.02239, -0.02239,
    0.0, -0.96, 1.16, 0.0, -0.3, 0.0, 0.02239, -0.02239
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

############################ Helper functions ############################

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

TASK_CONFIGS = {

    'aloha_mobile_hello_aloha':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_hello_aloha',
        'episode_len': 1000,
        'camera_names': ['cam_high']
    },

    'aloha_mobile_dummy':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_dummy',
        'episode_len': 1000,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    # wash_pan
    'aloha_mobile_wash_pan':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_wash_pan',
        'episode_len': 1100,
        'train_ratio': 0.9,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_wash_pan_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_wash_pan',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_wash_pan',
        ],
        'sample_weights': [5, 5],
        'train_ratio': 0.9, # ratio of train data from the first dataset_dir
        'episode_len': 1100,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    # wipe_wine
    'aloha_mobile_wipe_wine':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_wipe_wine',
        'episode_len': 1300,
        'train_ratio': 0.9,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_wipe_wine_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_wipe_wine',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_wipe_wine',
        ],
        'sample_weights': [5, 5],
        'train_ratio': 0.9, # ratio of train data from the first dataset_dir
        'episode_len': 1300,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_wipe_wine_2':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_wipe_wine_2',
        'episode_len': 1300,
        'train_ratio': 0.9, # ratio of train data from the first dataset_dir
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_wipe_wine_2_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_wipe_wine_2',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_wipe_wine_2',
        ],
        'sample_weights': [5, 5],
        'train_ratio': 0.9, # ratio of train data from the first dataset_dir
        'episode_len': 1300,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    # cabinet
    'aloha_mobile_cabinet':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_cabinet',
            DATA_DIR + '/aloha_mobile_cabinet_handles', # 200
            DATA_DIR + '/aloha_mobile_cabinet_grasp_pots', # 200
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_cabinet',
        ],
        'sample_weights': [6, 1, 1],
        'train_ratio': 0.99, # ratio of train data from the first dataset_dir
        'episode_len': 1500,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_cabinet_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_cabinet',
            DATA_DIR + '/aloha_mobile_cabinet_handles',
            DATA_DIR + '/aloha_mobile_cabinet_grasp_pots',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_cabinet',
        ],
        'sample_weights': [6, 1, 1, 2],
        'train_ratio': 0.99, # ratio of train data from the first dataset_dir
        'episode_len': 1500,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    # elevator
    'aloha_mobile_elevator':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_elevator',
        'train_ratio': 0.99,
        'episode_len': 8500,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_elevator_truncated':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_elevator_truncated',
            DATA_DIR + '/aloha_mobile_elevator_2', # 1200
            DATA_DIR + '/aloha_mobile_elevator_button', # 800
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_elevator_truncated',
            DATA_DIR + '/aloha_mobile_elevator_2',
        ],
        'sample_weights': [3, 3, 2],
        'train_ratio': 0.99, # ratio of train data from the first dataset_dir
        'episode_len': 2250,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_elevator_truncated_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_elevator_truncated',
            DATA_DIR + '/aloha_mobile_elevator_2',
            DATA_DIR + '/aloha_mobile_elevator_button',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_elevator_truncated',
            DATA_DIR + '/aloha_mobile_elevator_2',
        ],
        'sample_weights': [3, 3, 2, 1],
        'train_ratio': 0.99, # ratio of train data from the first dataset_dir
        'episode_len': 2250,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    # high_five
    'aloha_mobile_high_five':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_high_five',
        'train_ratio': 0.9,
        'episode_len': 2000,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_high_five_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_high_five',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_high_five',
        ],
        'sample_weights': [7.5, 2.5],
        'train_ratio': 0.9, # ratio of train data from the first dataset_dir
        'episode_len': 2000,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    # chair
    'aloha_mobile_chair':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_chair',
        'train_ratio': 0.95,
        'episode_len': 2400,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_chair_truncated':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_chair_truncated',
        'train_ratio': 0.95,
        'episode_len': 2000,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_chair_truncated_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_chair_truncated',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_chair_truncated',
        ],
        'sample_weights': [5, 5],
        'train_ratio': 0.95, # ratio of train data from the first dataset_dir
        'episode_len': 2000,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    # shrimp
    'aloha_mobile_shrimp':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_shrimp',
        'train_ratio': 0.99,
        'episode_len': 4500,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_shrimp_truncated':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_shrimp_truncated',
        'train_ratio': 0.99, # ratio of train data from the first dataset_dir
        'episode_len': 3750,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    'aloha_mobile_shrimp_truncated_cotrain':{
        'dataset_dir': [
            DATA_DIR + '/aloha_mobile_shrimp_truncated',
            DATA_DIR + '/aloha_compressed_dataset',
        ], # only the first dataset_dir is used for val
        'stats_dir': [
            DATA_DIR + '/aloha_mobile_shrimp_truncated',
        ],
        'sample_weights': [5, 5],
        'train_ratio': 0.99, # ratio of train data from the first dataset_dir
        'episode_len': 3750,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },
    # 'aloha_mobile_shrimp_2_cotrain':{
    #     'dataset_dir': [
    #         DATA_DIR + '/aloha_mobile_shrimp_2',
    #         DATA_DIR + '/aloha_mobile_shrimp_before_spatula_down', # 2200
    #         DATA_DIR + '/aloha_compressed_dataset',
    #     ], # only the first dataset_dir is used for val
    #     'stats_dir': [
    #         DATA_DIR + '/aloha_mobile_shrimp_2',
    #     ],
    #     'sample_weights': [5, 3, 2],
    #     'train_ratio': 0.99, # ratio of train data from the first dataset_dir
    #     'episode_len': 4500,
    #     'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    # },
}
