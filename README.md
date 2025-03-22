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


<div id="top"></div>

<h1 align="center">Pedestrian Avoidence Robot Car</h1>
<h4 align="center"></h4>
<!-- PROJECT LOGO -->:
<br />
<div align="center">

<h3>ECEMAE148 Final Project</h3>
<p>
Team 2 Winter 2025
</p>

<!-- ![image](https://github.com/JL2200/mae148_group8/blob/main/media/IMG_4898.JPG) -->
</div>


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#final-project">Final Project</a></li>
      <ul>
        <li><a href="#original-goals">Original Goals</a></li>
          <ul>
            <li><a href="#goals-we-met">Goals We Met</a></li>
            <li><a href="#our-hopes-and-dreams">Our Hopes and Dreams</a></li>
              <ul>
                <li><a href="#stretch-goal-1">Stretch Goal 1</a></li>
                <li><a href="#stretch-goal-2">Stretch Goal 2</a></li>
              </ul>
          </ul>
        <li><a href="#final-project-documentation">Final Project Documentation</a></li>
      </ul>
    <li><a href="#robot-design">Robot Design </a></li>
      <ul>
        <li><a href="#cad-parts">CAD Parts</a></li>
          <ul>
            <li><a href="#final-assembly">Final Assembly</a></li>
            <li><a href="#custom-designed-parts">Custom Designed Parts</a></li>
            <li><a href="#open-source-parts">Open Source Parts</a></li>
          </ul>
        <li><a href="#electronic-hardware">Electronic Hardware</a></li>
        <li><a href="#software">Software</a></li>
          <ul>
            <li><a href="#embedded-systems">Embedded Systems</a></li>
            <li><a href="#ros2">ROS2</a></li>
            <li><a href="#donkeycar-ai">DonkeyCar AI</a></li>
          </ul>
      </ul>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#authors">Authors</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- TEAM MEMBERS -->
## Team Members

<div align="center">
    <p align = "center">Andrew N, Daphne, Jose, and Rodolfo</p>
</div>

<h4>Team Member Major and Class </h4>
<ul>
  <li>Andrew N - Electrical Engineering, Curcuits and Systems - Class of 2026</li>
  <li>Daphne - Electrical Engineering, Curcuits and Systems - Class of 2026</li>
  <li>Jose - Biology Engineering </li>
  <li>Rodolfo - Mechanical Engineering</li>
</ul>

<!-- Final Project -->
## Final Project
<!-- put stuff here -->

<!-- Original Goals -->
### Original Goals
<!--Originally, we envisioned a chatgpt robot in a classroom that students could command to do tasks. Our goalpost task was "go to the whiteboard and help the student solve the problem." This would require chatgpt to navigate a room, create a path to a whiteboard, solve visual math problems by identifying text on a board, and provide its usefulness to a student through good help. -->
   
<!-- End Results -->
### Goals We Met
<!--We were succesfully able to communicate with the robot. We can ask chatgpt what it saw around it. Often chatgpt went into multiple paragraphs. One test we did was telling chat gpt to drive towards the hand with more fingers up. We held out our hand with 2 fingers to the left, and 4 fingers to the right. Chatgpt sent a drive command to turn towards the right. We also were able to generate decent paths with chatgpt; at one point we asked it to make a heart path and it followed the path pretty well. We feel that large language models open up many emergent capabilities for robots, and that our overall project of giving chatgpt a level of autonomy was a success. We feel that if we ran our original test, that chatgpt would do decently well, except for navigating around tables. Often times durring the debugging process we would just ask chatgpt what data it had. For example, when debugging lidar, we would ask it what it thought of the data format, what could be improved, and what reference data it wanted from the user. All in all, its linguistic capabilities were superb. -->

### Future Goals
#### Stretch Goal 1
<!-- We want to have chatgpt's path following trigger the manage.py drive command automatically so that chatgpt can navigate fully autonomous. We also want to have chatgpt only use one model instead of two seperated models. Finally, we want to turn the lidar data into a SLAM map and feed chatgpt an image map of its surroundings to generate better maps. -->

#### Stretch Goal 2
<!--We want automatic lidar stopping to be implemented for safety. Since chatgpt does not control the robot in real time, we need a way for the robot to stop if it is about to hit an object or person.-->

## Final Project Documentation

<!-- Early Quarter -->
### Robot Design CAD
<!--<img src="/media/full%20car%20cad.png" width="400" height="300" />-->

#### Open Source Parts
| Part | CAD Model | Source |
|------|--------|-----------|
<!--| Jetson Nano Case | <img src="/media/jetson%20nano%20case.png" /> | [Thingiverse](https://www.thingiverse.com/thing:3518410) | -->

### Software
#### Chatgpt
<!--We connected Chatgpt to the robocar by using the Openai API, utilizing two seperate models. The first model was GPT4 with Vision, which processes commands from the user and images. Then this model creates an action plan for what the car can do. Since the image based models don't have function calling to trigger the drive commands, we used a second GPT4-Turbo model to read the vision models plan and turn those into functions. Chatgpt had acces to two functions, a drive command to control steering, speed, and motion timeout. Then it had control over a path function, which let chatgpt generate a csv path of x and y coordinates + a throttle for the car to follow. Chatgpt had access to the cameras, lidar data, gps data, and user prompts. We picked and chose which data to give Chatgpt based on the use case.-->

#### Embedded Systems
<!--To run the system, we used a Jetson Nano with an Oakd depth camera, an ld06 lidar sensor, and a point one Fusion Engine gps. For motion we used a VESC Driver within the Donkey Car framework. https://www.donkeycar.com/-->

#### ROS2
<!--For commands, we made a ROS2 Node called ChatgptDriveSubpub that works with the UCSD Robocar framework. Most of the files we created are in the basics2 package of the ros2_ws (ros2 workspace). We altered the nav2 config files to add the chatgpt node to start up automatically, but never finished this. So, if you follow the steps to get Ros2 running from the UCSD robocar framework, you are mostly complete.-->

<!--In our project files, we had to add the fusion engine driver for gps manually, so the nodes for fusion gps are prone to error. One will need 4 to 5 terminals to run this system. One for starting up gps, one for launching all_nodes, one for launching the chatgpt node, one for sending chatgpt messeges over a chat topic, and finally one to use donkeycar's manage.py drive command to drive the car in a desired path. -->



<!--
### How to Run
Use the UCSD Robocar Docker images and add the projects folder yourself. Python3 is required, install any reposotries not there.

Step 1: In the first terminal, start the fusion client. First source ros2. Note, in the build_ros2 file, we exluded the build for the fusion client as each build takes an extra 50 seconds with it.

```source_ros2```

```build_ros2```

```ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tty -p tty_port:=/dev/ttyUSB1```


Step 2: In a second terminal, build and source ros2 as usual from the ucsd robocar process. Then launch all nodes.

```source_ros2```

```ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py```


Step 3: Adjust the chatgpt_drive_node.py in the basics package with your OPENAI API KEY. Launch the chatgpt node. Source ros2 in a third terminal and launch this command.

```source_ros2```

```ros2 launch ucsd_robocar_basics2_pkg chatgpt_drive_launch.launch.py```


Step 4: You are ready to talk to chatgpt. Make sure the terminals output no errors and chatgpt has said "finished init". Then, you publish to a /chat_input topic to communicate with the chatgpt node.

```source_ros2```

```ros2 topic pub -1 /chat_input std_msgs/msg/String "data: 'YOUR MESSEGE HERE'"```


The messege should show up in the chatgpt terminal and if it does not, it did not work. First the vision model will respond with an action plan. Responses can take up to 20 seconds. Then a Function tool caller will work. Sometimes the tool caller gets stuck and you must restart the chatgpt node. Its recommended to restart all nodes, as another error can be no image input being recieved from the all_nodes.

Step 5: If you desire a path to be made, then chatgpt will have outputed the path to the donkey_paths folder in the basics2 package. You can take this csv, and use its path in the donkey car manage.py config files for donkey car to drive on this desired path. Make sure your path starts at 0,0 or to zero the car and drive to the path start. The donkey command to run in a 5th terminal is

```manage.py drive ```

For more information, see the donkey car framework on running with your own car. When manage.py runs, use x to zero the car, b to load a path, and double tap start for the car to follow the path.

Here is a link which contains many photos and videos of our robot doing different chatgpt powered actions:
https://photos.google.com/u/0/share/AF1QipNbnRdItg1uGULNdo4saggAoKI3nbOa-YogXLWNfc9HYORkTFDVzfL07XVTVJIqRg?key=WGtpSk1jdEJXZjJ6cHpObENFQU9MSzItVG1jOXN3 
-->
<!-- Authors -->
## Authors
<!--
![image](https://github.com/UCSD-ECEMAE-148/winter-2024-team-2aka8/blob/main/media/148groupphoto.jpg)
-->
<!-- Badges -->
<!--
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
-->

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
<!--
*Thank you to my teammates, Professor Jack Silberman, and our incredible TA Arjun Naageshwaran for an amazing Winter 2024 class! Thank you Kiersten for the amazing readme template.*
-->

<!-- CONTACT -->
## Contact
<!--
* Jason | yul202@ucsd.edu
* Jesse | jerupe@ucsd.edu 
* Maahir | masgharali@ucsd.edu
* Alexander | ahaken@ucsd.edu
-->
