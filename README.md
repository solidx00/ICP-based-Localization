# ICP-based Localization

📌 Localizer2D - Robot Localization using ICP in ROS

Localizer2D is a robot localization system based on ICP (Iterative Closest Point) developed in ROS (Robot Operating System).
It uses laser scanner data and a map to estimate the robot's position in a probabilistic way.

✅ ICP (Iterative Closest Point) for scan-to-map alignment.
✅ KD-Tree for fast obstacle search.
✅ ROS Publisher/Subscriber for real-time communication.
✅ RViz support for visualization.

Launch RViz for visualization

roslaunch icp_localizer launchfile.launch
