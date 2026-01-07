from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description() -> LaunchDescription:
    # Get workspace root by going up from install directory
    # When installed: /install/corgi_panel/share/corgi_panel/launch/this_file.py
    # Path breakdown: launch/ -> corgi_panel/ -> share/ -> corgi_panel/ -> install/ -> workspace/
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Go up: launch -> share/corgi_panel -> share -> install/corgi_panel -> install -> workspace
    workspace_root = os.path.abspath(os.path.join(launch_dir, '..', '..', '..', '..', '..'))
    
    # Build path to source script
    script_path = os.path.join(
        workspace_root, 'src', 'corgi_panel', 'scripts', 'corgi_config_panel_dev.py'
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[script_path],
            output='screen',
        ),
    ])
