# ICP-based Localization

📌 Localizer2D - Robot Localization using ICP in ROS

Localizer2D is a robot localization system based on ICP (Iterative Closest Point) developed in ROS (Robot Operating System).
It uses laser scanner data and a map to estimate the robot's position in a probabilistic way.

✅ ICP (Iterative Closest Point) for scan-to-map alignment.
✅ KD-Tree for fast obstacle search.
✅ ROS Publisher/Subscriber for real-time communication.
✅ RViz support for visualization.

### Dipendenze
Per eseguire il progetto, assicurati di avere installati:
```bash
ROS (Noetic/Melodic/Foxy per ROS2)
Eigen (Libreria per algebra lineare)
OpenCV (Per la gestione delle mappe)
PCL (Point Cloud Library) (Facoltativo per visualizzazione avanzata)
```

### Launch RViz for visualization
```bash
roslaunch icp_localizer launchfile.launch
```

### ROS Topics
| Topic | Message Type    | 	Description 
| :---:   | :---: | :---: | 
| /map | nav_msgs/OccupancyGrid   | Environment map 
| /initialpose | geometry_msgs/PoseWithCovarianceStamped   | Initial robot pose 
| /base_scan | 	sensor_msgs/LaserScan   | Laser scanner data 
| /odom_out | nav_msgs/Odometry   | Estimated robot pose 
| /scan_out | sensor_msgs/LaserScan   | Transformed laser scan 
