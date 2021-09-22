# suturo_manipulation

This repo contains all necessary package to have the Robot interract with its enviroment. The main focus is on the arm of the objects and focuses on mainly two objectives. Grasb and place objects given a position.

Additinally a package is included which positions the arm of the robot in certain predifined poses making sure the sensors of the robot are not blocked by itself.

Further deatails regarding each package can be found in its corresponding readme.

To start the manipulation servers run:

`roslaunch suturo_manipulation_launch start_manipulation_servers.launch`

You can run:

`rosrun int_test_marker int_tes_marker.py`

This will publish an interactive marker u can use to grasp, place at certain position. Additionaly you can select pick a pose the robot shall take. 

## Visual Servoing (Inspired by https://github.com/osrf/tensorflow_object_detector)

**Make sure to install `tensorflow` version 1.14.0. Also copy the modified `hsrb_gazebo_bringup` package mentioned in the Projektdoku into the `src` folder of your workspace. Due to legal reasons, it is not uploaded to any SUTURO repository.** 

To use the Visual Servoing, checkout the repositories of Giskard, Suturo_Manipulation and Suturo_Resources to the visual servoing branch.
After that one can start the visual servoing by the provided launch file:

`roslaunch suturo_bringup visual_servoing_bringup.launch`

Developing for visual servoing requires a `virtualenv` setup, because of the usage of python 3.6+. Follow the instructions in https://github.com/osrf/tensorflow_object_detector to setup the environment
