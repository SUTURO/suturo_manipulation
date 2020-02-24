# suturo_manipulation

This repo contains all necessary package to have the Robot interract with its enviroment. The main focus is on the arm of the objects and focuses on mainly two objectives. Grasb and place objects given a position.

Additinally a package is included which positions the arm of the robot in certain predifined poses making sure the sensors of the robot are not blocked by itself.

Further deatails regarding each package can be found in its corresponding readme.

To start the manipulation servers run:

roslaunch suturo_manipulation_launch start_manipulation_servers.launch 

You can run:

rosrun int_test_marker int_tes_marker.py

This will publish an interactive marker u can use to grasp, place at certain position. Additionaly you can select pick a pose the robot shall take. 
