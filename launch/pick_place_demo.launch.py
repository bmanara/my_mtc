from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder(robot_name="ur")
        .robot_description_semantic(file_path="/home/brian_2025/Github/moveit_ws/install/ur_moveit_config/share/ur_moveit_config/srdf/ur.srdf.xacro", mappings={"name": "ur", "ur_type": "ur10"})
        .robot_description_kinematics(file_path="/home/brian_2025/Github/moveit_ws/install/my_ur/share/my_ur/config/kinematics.yaml")
        .robot_description(file_path="/home/brian_2025/Github/moveit_ws/install/my_ur/share/my_ur/urdf/ur.urdf.xacro", mappings={"name": "ur", "ur_type": "ur10"})
        .joint_limits(file_path="/home/brian_2025/Github/moveit_ws/install/ur_moveit_config/share/ur_moveit_config/config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="/home/brian_2025/Github/moveit_ws/install/ur_moveit_config/share/ur_moveit_config/config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="/home/brian_2025/Github/moveit_ws/install/ur_moveit_config/share/ur_moveit_config/config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_dict()
    )

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "state_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MTC Demo node
    pick_place_demo = Node(
        package="my_mtc",
        executable="my_mtc",
        output="screen",
        parameters=[
            ompl_planning_pipeline_config,
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
