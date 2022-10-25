from typing import List

MOVE_GROUP_ARM: str = "lite6"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

# https://github.com/xArm-Developer/xarm_ros2/blob/master/xarm_moveit_config/srdf/_lite6_macro.srdf.xacro
def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "world_joint",
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
        prefix + "joint_eef",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "link_base"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "link_eef"


def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "link_eef",
        prefix + "link_eef",
    ]
