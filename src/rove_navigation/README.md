# Rove Navigation

This package contains navigation-related nodes and configurations for the Rove robot.

## Launch Files

- **navigation.launch.py**: Launches the navigation stack, including path planning and obstacle avoidance.
- **mule_behaviour.launch.py**: Launches the mule behavior node, which implements specific behaviors for the robot.

## Nodes

- **frontier_publisher**: Publishes frontier points for exploration, helping the robot to explore unknown areas.
- **ovis_pull**: Handles Ovis position locking, ensuring that the robot maintains a stable position.
- **person_following**: Follows a person, using sensors to track and follow a human target.
- **mule_behavior**: Implements mule behavior, allowing the robot to carry and transport objects.