# GeiBike Project

The GeiBike project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous vehicule, base on a 3-wheels bike, in order to carry out different missions. Several projects are exposed on the [official website](https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiBike. The present code as well as the documentation is the result of the combination of the various projects carried out by:

* TBD

The platform is (or was) developped and maintained by :

* LOMBARD Emmanuel
* MARTIN José
* BOUBACAR ALZOUMA Abdou Rahamane 
* DI MERCURIO Sébastien


The projects are (or were) surpervised by:

* CHANTHERY Elodie
* AURIOL Guillaume

## Quick User Guide


### Installation
Clone the repo
Go to catkin_ws folder 
Git clone https://github.com/dusty-nv/ros_deep_learning.git
Go back to /catkin_ws
catkin_make
source devel/setup.bash
3 prompts : 
roscore     
roslaunch ros_deep_learning detectnet.ros1.launch input:=/dev/video0 output:=display://0
rosrun ros_recognition reco_ros.py 



