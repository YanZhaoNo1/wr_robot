# ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
# ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/zy/ws/slam_ws/map/mymap.pbstream'}"
# ros2 run cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/zy/ws/slam_ws/map/mymap -pbstream_filename=/home/zy/ws/slam_ws/map/mymap.pbstream -resolution=0.05

#!/bin/bash

# 检查是否提供了文件名前缀参数
if [ -z "$1" ]; then
    echo "Usage: . map_save.sh <map_name>"
    return 1
fi

MAP_NAME=$1
MAP_DIR="/home/zy/ws/wr_robot/src/ucar_navigation/maps"
PBSTREAM_FILE="$MAP_DIR/$MAP_NAME.pbstream"
MAP_FILESTEM="$MAP_DIR/$MAP_NAME"

# 结束当前的 Cartographer 轨迹
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# 保存 Cartographer 生成的 pbstream 地图文件
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$PBSTREAM_FILE'}"

# 将 pbstream 转换为 ROS 地图格式
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -map_filestem=$MAP_FILESTEM \
    -pbstream_filename=$PBSTREAM_FILE \
    -resolution=0.05

# 提示用户保存完成
echo "Map saved as $MAP_FILESTEM.pgm and $MAP_FILESTEM.yaml"
