ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false 
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
  # In a new terminal on your laptop (sourced)
# Save it into your package's src/maps directory for version control
ros2 run nav2_map_server map_saver_cli -f ~/CBL/NutrientNexus/src/my_turtlebot3_controller/maps/lab_map --ros-args -p use_sim_time:=false


ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=false \
    autostart:=true \
    map:=/home/yassin/CBL/NutrientNexus/src/my_turtlebot3_controller/maps/big_map.yaml \
    params_file:=/home/yassin/CBL/NutrientNexus/src/my_turtlebot3_controller/config/nav2_real_params.yaml
