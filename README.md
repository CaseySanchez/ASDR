# ADR

# Launch RealSense
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# Launch RTABMAP mapping
roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false

# View saved RTABMAP map    
rtabmap-databaseViewer ~/.ros/rtabmap.db

# Launch RTABMAP localization
roslaunch rtabmap_ros rtabmap.launch localization:=true
