# **RoboND Kinematics Project: Pick & Place**

[//]: # (Image References)

[img_pipeline]: _imgs/img_pickAndPlace_pipeline_1.png
[img_leojs_demo_scene]: _imgs/img_webtool_demo_scene.png
[img_leojs_demo_playground]: _imgs/img_webtool_demo_playground.png
[img_fk_test_launch]: _imgs/img_fk_test_launch.png
[img_joints_from_kr210_urdf_1]: _imgs/img_joints_from_kr210_urdf_1.png
[img_joints_from_kr210_urdf_2]: _imgs/img_joints_from_kr210_urdf_2.png
[img_joint_definition]: _imgs/img_joint_definition.png
[img_link_definition]: _imgs/img_link_definition.png

[gif_fk_test]: _imgs/gif_fk_test.gif

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## **About the project**

This project consists of implementing the Inverse Kinematics of the KUKA KR210 manipulator, in order to perform a pick and place task in a simulated environment.

The implementation is mainly in ROS, using the following tools :

*   **Gazebo**, for simulation of the pick and place task.
*   **RViz**, to visualize the kinematics of the manipulator.
*   **MoveIt**, to make planning actions and get the trajectories we must follow.

The work done in this project consists on doing the **Kinematic Analysis** of the Kuka KR210 robot in order to implement the **IK_server** ROS node, which will be in charge of giving a service for the environment to request joint trajectories, given the pose trajectories returned by the planner.

As a bonus, I implemented a web tool in typescript which helped me in this analysis, as well as to understand some of the low level details that you run into when trying to implement something from scratch.

![leojs_demo_scene][img_leojs_demo_scene]

![leojs_demo_playground][img_leojs_demo_playground]

## **Kinematic Analysis**

### Extracting the Denavit Hantenberg ( DH ) parameters

To check the structure of the manipulator, we ran the FK_test.launch in ROS and checked the structure of the manipulator in RViz.

![fk test][img_fk_test_launch]

From the test, we can see that the structure of the robot consists of **6 revolute joints** ( to control the Forward kinematics ) and **2 prismatic joints** ( to control the gripper open-close operations ). The data for these joints can be found in the **kr210.urdf.xacro** file, which contains the definition of the KR210 manipulator.

![urdf joints data 1][img_joints_from_kr210_urdf_1]
![urdf joints data 2][img_joints_from_kr210_urdf_2]

The important data to construct the DH representation of the manipulator consists of :

*   **parent** : link in which this joint is defined ( respect to ).
*   **child**  : link which the joint affects, respect to the parent's frame.
*   **origin** : pose of joint respect to parent's frame.
*   **axis**   : direction of the joint axis respect to parent's frame.

The information can look a bit redundant if you already know what the DH representation should look like ( as we already saw part of it in the lecture videos ), but if you haven't, this information will define the kinematic structure of the manipulator ( this information is used by ROS to define a **kinematic tree**, which is the one used in the **FK_test** demo ).

To understand how to interpret this data, we can go to the ROS documentation on **links** and **joints** for urdf files, where we can find thess helpful images :

![link urdf][img_link_definition]
![joint urdf][img_joint_definition]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


