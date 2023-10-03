## How to run the YOLO verification

1. In one terminal run your rosbag
```
rosbag play  ~/catkin_ws/levelb_pepper_occlusion_2023-10-03-18-12-51.bag 
```

2. In another terminal run 

```
rosrun visual_servo perception.py 
```

3. To record yolo run results, 

```
rosbag record /pepper_yolo_results -o ~/catkin_ws/log/best4-levelbepper_occlusion_2023-10-03-18-12-51/
```