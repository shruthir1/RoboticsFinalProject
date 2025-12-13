import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    final_project_dir = get_package_share_directory('finalProject') # Replace 'finalProject' with your actual package name

    # Define the Navigation Node (from navRevised.cpp)
    navigation_node = Node(
        package='finalProject',  # Your package name
        executable='navigation_node', # The executable name defined in CMakeLists.txt (from your main)
        name='navigation_node',
        output='screen',
        # Set parameters (e.g., debug/verbose flags if needed)
        # parameters=[{'debug': True, 'verbose': False}] 
    )

    # Define the Perception Node (from perception_node.cpp)
    perception_node = Node(
        package='finalProject',  # Your package name
        executable='perception_node', # The executable name defined in CMakeLists.txt
        name='perception_node',
        output='screen',
        # Important: Ensure /scan and /map topics are accessible
    )

    # Create the launch description and populate it
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(navigation_node)
    ld.add_action(perception_node)

    return ld