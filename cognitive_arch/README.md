# COGNITIVE ARCH

## Description

<p align="justify">
In this project we have to create a blackboard to add knowledge and connected it with a state machine which consists on explore all the house.
</p>

<p align="justify">
This is the diagram of the robot exploration 
</p>

![Image text](https://github.com/Juancams/plan_sist_cogn/blob/main/resources/states.png)


<p align="justify">
This is an image of the communication between the nodes and the blackboard
</p>

![Image text](https://github.com/Juancams/plan_sist_cogn/blob/main/resources/blackboard.jpg)


<p align="justify">
You can watch the video here
</p>

[![Watch the video](https://github.com/Juancams/plan_sist_cogn/blob/main/resources/init_cap.png)](https://youtu.be/BttqsvfpFCs)

## How to use?

```console
ros2 launch cognitive_arch cognitive_launch.py
ros2 run blackboard blackboard_main
ros2 run cognitive_arch hfsm_main
```
