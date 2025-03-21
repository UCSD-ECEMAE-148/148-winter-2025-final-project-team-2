# ucsd_robocar_hub2

<img src="ucsd_ros2_logos.png">

This is the UCSD robocar hub metapackage. It contains packages for our sensors, actuators, independent path planning algorithms and the navigation package that functions as the "brain" by being able to call all of the other packages and do some interesting things by combining multiple types of path planning and the sensors used to accomplish this. All packages are written in **ROS2** and have been tested on **Foxy**.

<div align="center">

# Getting Started 

</div>

1. Its suggested to just pull the <a href="https://hub.docker.com/r/djnighti/ucsd_robocar" >docker image</a> for this metapackage for faster development instead of having to install all the (many) dependencies required to get this metapackage working.
1. Next, go to the <a href="https://docs.google.com/document/d/1YS5YGbo8evIo9Mlb0J-w2r3bZfju37Zl4UmdaN2CD2A/edit?usp=sharing" >UCSD Robocar Framework Guidebook</a> to get an understanding of how to use the frameowork which also has step-by-step instructions for getting the camera navigation working and plenty of other details.
1. Each package has its own dedicated README.md that explains in more detail how it works and what launch/nodes/topics it has to offer. To get to each of the packages, just click on any of the submodules above. 

<div align="center">

## **Tools**

</div>

### UCSD Robocar Framework Guide Book

The UCSD Robocar framework has its own user manual that explains in detail how it works and how to use it. It also includes a step-by-step tutorial for calibrating camera, steering and throttle parameters for lane detection and following.

<a href="https://docs.google.com/document/d/1YS5YGbo8evIo9Mlb0J-w2r3bZfju37Zl4UmdaN2CD2A/edit?usp=sharing" >**UCSD Robocar Framework Guide Book**</a>


### ROS2 Guide Book

For help with using ROS in the terminal and in console scripts, check out this google doc below to see plenty of examples of using ROS2!

<a href="https://docs.google.com/document/d/1DJgVLnu_vN-IXKD3QrQVF3W-JC6RiQPVugHeFAioB58/edit?usp=sharing" >**ROS2 Guide Book**</a>


<div align="center">

## **Updating Repos**

</div>

Adding new packages: 
1. `git submodule add <remote_url>`
1. `git commit -m "message"`
1. `git push`

Updating packages: 
1. If local changes have been made, the command below will fail unless you commit (`git add .` then `git commit -m "message"`) or stash (`git stash`) them! 
1. `git submodule update --remote --merge` **Pay attention to the output of this command, to make sure it did not fail or Abort...**

Removing packages:
1. `git submodule deinit <submodule>`
1. `git rm <submodule>`

<div align="center">

## Troubleshooting

</div>

[**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg) : see troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_lane_detection2_pkg/-/tree/master#troubleshooting" >here</a>

[**ucsd_robocar_sensor2_pkg**](#ucsd-robocar-sensor2-pkg) : see troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_sensor2_pkg#troubleshooting" >here</a>

[**ucsd_robocar_actuator2_pkg**](#ucsd-robocar-actuator2-pkg) : see troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg#troubleshooting" >here</a>
