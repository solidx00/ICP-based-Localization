# ICP-based Localization

📌 Localizer2D - Robot Localization using ICP in ROS

Localizer2D is a robot localization system based on ICP (Iterative Closest Point) developed in ROS (Robot Operating System).
It uses laser scanner data and a map to estimate the robot's position in a probabilistic way.

✅ ICP (Iterative Closest Point) for scan-to-map alignment.

✅ KD-Tree for fast obstacle search.

✅ ROS Publisher/Subscriber for real-time communication.

✅ RViz support for visualization.

### Project Architecture
| File | Description
| :---:   | :---: | 
| localizer_node.cpp | Nodo ROS principale che gestisce le comunicazioni.  
| localizer2d.cpp | Implementa l'algoritmo ICP per la localizzazione. 
| map.h/.cpp | 	Gestione della mappa di occupazione. 
| ros_bridge.h/.cpp | Converte i dati tra ROS e Eigen.
| icp/eigen_icp_2d.h/.cpp | Algoritmo ICP per allineare il laser scan con la mappa.

### 📌 Dependencies
- Per eseguire il progetto, assicurati di avere installati:
- ROS (Noetic/Melodic/Foxy per ROS2)
- Eigen (Libreria per algebra lineare)
- OpenCV (Per la gestione delle mappe)
- PCL (Point Cloud Library) (Facoltativo per visualizzazione avanzata)

### Installation of dependencies
```bash
sudo apt update
sudo apt install ros-noetic-tf2-ros ros-noetic-nav-msgs ros-noetic-sensor-msgs
sudo apt install libeigen3-dev libopencv-dev
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
