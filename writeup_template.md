---
layout : notes
---

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
[img_joints_frames]: _imgs/img_joints_frames.png
[img_dh_representation_1]: _imgs/img_dh_representation_1.png
[img_dh_representation_2]: _imgs/img_dh_representation_2.png

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

### **1. Extracting the Denavit Hantenberg ( DH ) parameters**

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

The information can look a bit redundant if you already know what the DH representation should look like ( as we already saw part of it in the lecture videos ), but if you haven't, this information will define the kinematic structure of the manipulator ( this information is used by ROS to define a **kinematic tree** under the hood, which is the one used in the **FK_test** demo ).

To understand how to interpret this data, we can go to the ROS documentation on **links** and **joints** for urdf files, where we can find thess helpful images :

![link urdf][img_link_definition]
![joint urdf][img_joint_definition]

With this in mind we can construct the following diagram, which represents the joints' frames and their relative poses.

![joint frames][img_joints_frames]

From the way the frames are defined in ROS, the frames we saw in the RViz figure before are the joint-frames, but could be also thought as the link-frames ( the starting frame of reference of a link is the joint frame, and then every other frame, like visual or collision frames, are defined respect to this frame ).

Of course, this frames are not the actual frames of the DH representation, as they are not compliant with the assignment algorithm for the DH representation; but still, if we wanted we could use these to get the Forward Kinematics of the manipulator ( as we have all the information needed to construct transformations between these frames ), it's just that the DH representation is a handy way to think of these frames a bit more *'programmatically'*.

So, to get the DH representation we start with the previous information ( relative distances and joint axis ) and assign the frames for the DH representation, which yields the following structure:

![dh frames][img_dh_representation_1]

The frames have different positions and orientations relative to the previous joint frames, because of the DH assignment algorithm, and we can see that there is a relative orientation of the **End Effector** frame in the DH representation respect to the one in the previous joint frames representation, which generates the compensation matrix we saw in the lectures.

Also, here is the representation in the web simulator ( up to last dh frame ):

![dh frames leojs][img_dh_representation_2]

Finally, we can derive the DH parameter table using the previous information for the DH representation, which yields the following table: 

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
----- | ---------- | ------ | ------ | --------
0->1  |      0     |   0    | 0.75   | q1
1->2  |   -pi/2    |  0.35  | 0      | -pi/2 + q2
2->3  |      0     | 1.25   | 0      | q3
3->4  |   -pi/2    | -0.054 | 1.5    | q4
4->5  |    pi/2    |   0    | 0      | q5
5->6  |   -pi/2    |   0    | 0      | q6
6->EE |     0      |   0    | 0.303  | 0

### **2. Derivation of joint-joint transforms and total End Effector transform**

Recall the DH transformation matrix from frame *i-1* to frame *i*.

<img src="https://latex.codecogs.com/gif.latex?\fn_cm&space;T_{i-1}^{i}&space;=&space;\begin{bmatrix}&space;\cos(\theta_{i})&space;&&space;-\sin(\theta_{i})&space;&&space;0&space;&&space;a_{i-1}\\&space;\sin(\theta_{i})&space;\cos(\alpha_{i-1})&space;&&space;\cos(\theta_{i})&space;\cos(\alpha_{i-1})&space;&&space;-\sin(\alpha_{i-1})&space;&&space;d_{i}&space;\sin(\alpha_{i-1})\\&space;\sin(\theta_{i})&space;\sin(\alpha_{i-1})&space;&&space;\cos(\theta_{i})&space;\sin(\alpha_{i-1})&space;&&space;\cos(\alpha_{i-1})&space;&&space;d_{i}&space;\cos(\alpha_{i-1})\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}" title="T_{i-1}^{i} = \begin{bmatrix} \cos(\theta_{i}) & -\sin(\theta_{i}) & 0 & a_{i-1}\\ \sin(\theta_{i}) \cos(\alpha_{i-1}) & \cos(\theta_{i}) \cos(\alpha_{i-1}) & -\sin(\alpha_{i-1}) & d_{i} \sin(\alpha_{i-1})\\ \sin(\theta_{i}) \sin(\alpha_{i-1}) & \cos(\theta_{i}) \sin(\alpha_{i-1}) & \cos(\alpha_{i-1}) & d_{i} \cos(\alpha_{i-1})\\ 0 & 0 & 0 & 1 \end{bmatrix}" />

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


