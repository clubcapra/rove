# Setup rove_zed

Être sûr:
1. D'avoir la zed de branchée (port usb d'en dessous, celui du dessus ne fonctionne pas)
2. D'avoir le lidar de branché, 12V + ethernet (FAIRE ATTENTIONQUE LE CABLE FONCTIONNE ET EST BIEN BRANCHÉ)
3. D'avoir CUDA 12 d'installé et que zed Diagnostics fonctionne (`/usr/local/zed/tools/ZED_Diagnostic`)
4. D'avoir le network profile mis à `LIDAR profile` (IP adress `192.168.84.150`, aller dans settings > network)

## Pour démarrer le mapping rtabmap avec la zed:
```bash
ros2 launch rove_zed zed_mapping.launch.py
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    database_path:="/media/SSD/stable/rove/src/rove_rtabmap/maps/map.db" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/depth/camera_info \
    odom_topic:=/zed/zed_node/odom \
    imu_topic:=/zed/zed_node/imu/data \
    visual_odometry:=false \
    frame_id:=zed_camera_link \
    approx_sync:=false \
    rgbd_sync:=true \
    approx_rgbd_sync:=false
```

## Pour démarrer le body tracking:
```bash
ros2 launch rove_zed body_trck.launch.py
```

## Pour démarrer le lidar:
```bash
ros2 launch rove_bringup test.launch.py
```

## Si le launch file de la zed ne fonctionne pas:

Appeler zed_wrapper directement avec:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```
---


# Rove Zed 

This package contains configurations and launch files for the Zed camera.

## Launch Files

- **zed_body_trck.launch.py**: Launches the body tracking with the Zed camera, enabling the robot to track and follow human bodies.
- **zed_mapping.launch.py**: Launches the mapping with the Zed camera, creating a 3D map of the environment.

## Nodes

- **zed_camera**: Handles the Zed camera operations, including capturing images and depth data.

## Usage

To start the mapping with RTABMAP and the Zed camera, use the following commands:

```bash
ros2 launch rove_zed zed_mapping.launch.py
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    database_path:="/media/SSD/stable/rove/src/rove_rtabmap/maps/map.db" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/depth/camera_info \
    odom_topic:=/zed/zed_node/odom \
    imu_topic:=/zed/zed_node/imu/data \
    visual_odometry:=false \
    frame_id:=zed_camera_link \
    approx_sync:=false \
    rgbd_sync:=true \
    approx_rgbd_sync:=false