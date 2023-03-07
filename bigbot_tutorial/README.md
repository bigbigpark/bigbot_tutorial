# bigbot Project

This is my personal tutorial but any question is welcoming :)

I'll use **one** and **multiple** robots for my project

I used rosbot 2.0

<br/>

**OS**: Ubuntu 18.04 (ROS melodic)

**CPU**: i9-9th

**GPU**: RTX 2080

<br/>

<hr>

## 1. Install rosbot description

~~~bash
$ git clone https://github.com/bigbigpark/rosbot_description.git
~~~

Then,

~~~bash
$ cd ${WORKING_DIRECTORY} && catkin build
~~~

<br/>

## 2. Clone my project

~~~bash 
$ git clone https://github.com/bigbigpark/bigbot_tutorial.git
~~~

Again,

~~~bash
$ cd ${WORKING_DIRECTORY} && catkin build
~~~

<br/>

## 3. Run code

There are two different mode here <br/>
One is `one_robot.launch` and the other is `robots.launch`
You can run one robot with gazebo+rviz,
~~~bash
$ roslaunch bigbot_tutorial bigbot_gazebo.launch rviz:=true
~~~
<hr>

## 1. Turning Control

I have implemented turning control using LOS & CTE errors <br/>

~~~bash
$ rosrun bigbot_tutorial turningControl
~~~





<hr>

## Future work

* **Dead Reckoning** using **/odom** from robot's encoder on gazebo
* **EKF**-based odometry estimation using **DR and IMU data**

* **2D ICP** using laser scanner equipped on rosbot
* Run SOTA **VO** algorithm using depth topic
* Formation control for multiple robots
* (IDK) **Reinforcement learning** about cooperative control



<br/>

