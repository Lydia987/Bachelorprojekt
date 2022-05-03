

# General Info
The "follow_pkg"-ROS package we developed, makes it possible that the jackal autonomously follows a human in simulation and in the field.
The person the robot is to follow carries a smartphone, whose GPS data is sent to the Jackal using the ROS-Mobile app.
Together with its own GPS position and orientation, the latter calculates the direction in which it must travel.
Other behaviors such as Avoid and Stop are used for collision avoidance and safety and improve the overall behavior.

***
Attention the topic names for the simulation version are different than for the real version.
At the moment the topic names are used for the execution in simulation.
For the execution in the real world the corresponding commented code lines must be reactivated and
the code lines for the execution in simulation must be commented out.
***

Used versions of ROS and Ubuntu: Ubuntu 16, ROS kinetc\
For more information, please check out our package documentation.

# Used Technologies
To simulate the jackal, we used the jackal_simulator and jackal_desktop packages.
In order to get a more accurate GPS position of the Jackal we have used the robot_localization package.
In order to get a NavSatFix message from the smartphone we used the ROS-Mobile app and implemented a new widget (smartphonegps).
To simulate GPS in gazebo, we used the hector_gazebo_plugin.
In order to simulate a smartphone that publishes a NavSatFix message we build the GPS_Ball modell.
To test how the Jackal reacts to different obstacles we used models from the gazebo_models.

## jackal_simulator
* http://wiki.ros.org/jackal_simulator
* https://github.com/jackal/jackal_simulator/tree/kinetic-devel

## jackal_desktop
* http://wiki.ros.org/jackal_desktop
* https://github.com/jackal/jackal_desktop/tree/kinetic-devel

## ROS-Mobile
* without our new widget: https://github.com/ROS-Mobile/ROS-Mobile-Android
* with our new widget: app-release.apk

## robot_localization
* http://wiki.ros.org/robot_localization
* https://github.com/cra-ros-pkg/robot_localization/tree/kinetic-devel

## hector_gazebo_plugins
* http://wiki.ros.org/hector_gazebo_plugins
* https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/tree/kinetic-devel

## GPS_Ball
look in our models folder

## gazebo_models
https://github.com/osrf/gazebo_models

# How to creat a new behavior
To implement a new behavior, you need to write a new Python class that publishes twist messages on a topic with the name of your behavior.
This topic must then be subscribed by the arbiter.
Also you need to implement in the run method of the arbiter when this behavior is activated or how it is combined with other existing behaviors.
