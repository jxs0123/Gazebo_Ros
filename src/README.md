roslaunch arm_gazebo sim_bringup.launch

roslaunch arm_gazebo spawn_object.launch

conda activate yolov5 && roslaunch yolov8_ros yolo_v8.launch

roslaunch grasp_demo start_server.launch

rosrun grasp_demo obj_grasp.py