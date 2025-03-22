# UCSD ECEMAE148 Team2 FinalProject

<img src="ucsd_ros2_logos.png">
<img src="ublox.jpg" width="300" height="300">
<div>
<h3>This is the github repo for Team2, we mainly focused on recovering Ublox GPS in ROS2. </h3>
<body>
  <p>
  In the original github repository from <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2">ucsd_robocar_hub2</a>, but it dosen't have the functionality of running gps inside the docker container(ROS2). Therefore, we implemented the node for ublox gps from scratch to work with the existing files for the robot to follow gps coordinates. It would be useful for future classes.
  </p>
  <p>
  To get started, click into /ucsd_robocar_hub2 folder and you can see a lot of packages listed. This is the main flow of the nodes interect with each other when we tested the gps: (placeholder of the block diagram).
  </p>
  <p>As mentioned in the final presentation, we found out the original packages is missing a ublox gps node to connect to the gps, therefore we added a node called "ublox_gps_node.py" under the directory ucsd_robocar_hub2/ucsd_robocar_sensor2_pkg/ucsd_robocar_sensor2_pkg/, as well as did a series of testing to see if the node was interecting with the others while sending correct data.</p>
</body>
