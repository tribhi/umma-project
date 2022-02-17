roslaunch rtabmap_ros rtabmap.launch \
rtabmap_args:=”–delete_db_on_start” \
depth_topic:=/robot_1_depth \
rgb_topic:=/robot_1_rgb \
camera_info_topic:=robot_1_camera_info_topic \
approx_sync:=false \
rtabmapviz:=false \
