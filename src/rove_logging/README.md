# Rove_logging

This package serves as a callable set of launch files to control the logging of ros bags of the robot. All files are outputted to ./output/bags

## start the rosbag logging
```bash
ros2 launch rove_logging start_logging.launch.py
```

## compress the ouput folder to ouput.zip 
> only includes the latest rosbag record + the rest of the folder without the readme
```bash
ros2 launch rove_logging compress_output.launch.py
```