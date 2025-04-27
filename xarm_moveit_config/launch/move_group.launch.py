from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("xarm", package_name="xarm_moveit_config").to_moveit_configs()
    
    # 设置 MoveIt 配置
    moveit_config.move_group_capabilities = {
        "capabilities": ["move_group/ExecuteTaskSolutionCapability"],  # 列表（可迭代）
        "disable_capabilities": "False"  # 改为字符串
    }
    
    return generate_move_group_launch(moveit_config)