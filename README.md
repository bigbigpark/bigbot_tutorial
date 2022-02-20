
# My bigbot tutorial

<br/>

~~~bash
$ sudo apt install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation
~~~

<br/>

~~~bash
$ roslaunch jackal_gazebo jackal_world.launch
$ roslaunch jackal_gazebo jackal_world.launch config:=front_laser
$ roslaunch jackal_viz view_robot.launch
$ roslaunch jackal_control teleop.launch joy_dev:=/dev/input/js0
~~~
