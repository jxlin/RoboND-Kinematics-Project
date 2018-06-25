
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

[img_tf_i_i_1]: _imgs/img_tf_i_i_1.png
[img_tf_6_5]: _imgs/img_tf_6_5.png
[img_tf_5_4]: _imgs/img_tf_5_4.png
[img_tf_4_3]: _imgs/img_tf_4_3.png
[img_tf_3_2]: _imgs/img_tf_3_2.png
[img_tf_2_1]: _imgs/img_tf_2_1.png
[img_tf_1_0]: _imgs/img_tf_1_0.png

[img_total_transform_1]: _imgs/img_total_transform_1.png
[img_total_transform_2]: _imgs/img_total_transform_2.png
[img_total_transform_3]: _imgs/img_total_transform_3.png
[img_total_transform_4]: _imgs/img_total_transform_4.png
[img_total_transform_5]: _imgs/img_total_transform_5.png

[img_tf_eef_compensation]: _imgs/img_tf_eef_compensation.png
[img_tf_eef_base]: _imgs/img_tf_eef_base.png

[img_tf_3_0]: _imgs/img_tf_3_0.png
[img_tf_6_3]: _imgs/img_tf_6_3.png

[img_ik_problem_separation]: _imgs/img_ik_problem_separation.png
[img_ik_solver_algorithm]: _imgs/img_ik_solver_algorithm.png

[img_ik_decoupling_1]: _imgs/img_ik_decoupling_1.png
[img_ik_decoupling_2]: _imgs/img_ik_decoupling_2.png
[img_ik_decoupling_3]: _imgs/img_ik_decoupling_3.png

[img_ik_geometricSolution_case1]: _imgs/img_ik_geometricSolution_case1.png

[gif_fk_test]: _imgs/gif_fk_test.gif

[image3]: ./misc_images/misc2.png

## **About the project**

This project consists of implementing the Inverse Kinematics of the KUKA KR210 manipulator, in order to perform a pick and place task in a simulated environment.

The implementation is mainly in ROS, using the following tools :

*   **Gazebo**, for simulation of the pick and place task.
*   **RViz**, to visualize the kinematics of the manipulator.
*   **MoveIt**, to make planning actions and get the trajectories we must follow.

The work done in this project consists on doing the **Kinematic Analysis** of the Kuka KR210 robot in order to implement the **IK_server** ROS node, which will be in charge of giving a service for the environment to request joint trajectories, given the pose trajectories returned by the planner.

As a bonus I implemented a web tool in typescript which helped me in this analysis, as well as to understand some of the low level details that you run into when trying to implement something from scratch.

![leojs_demo_scene][img_leojs_demo_scene]

[foo bar fun](https://wpumacay.github.io/leoJS/index.html)

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

We haven't include the orientation compensation of the gripper frame because we will deal with it separately.

### **2. Derivation of joint-joint transforms and total End Effector transform**

Recall the DH transformation matrix from frame *i-1* to frame *i*.

![DH frames transformation][img_tf_i_i_1]

By replacing for each DH frame we get the following transforms :

![DH frames 1 to 0][img_tf_1_0]

![DH frames 2 to 1][img_tf_2_1]

![DH frames 3 to 2][img_tf_3_2]

![DH frames 4 to 3][img_tf_4_3]

![DH frames 5 to 4][img_tf_5_4]

![DH frames 6 to 5][img_tf_6_5]

Calculating intermediate transforms from *3 to 0* and *6 to 3* ( we will need them later for the inverse kinematics solution ) we get:

![dh frames 3 to 0][img_tf_3_0]

![dh frames 6 to 3][img_tf_6_3]

Combining these we get the total transform from frame 6 respect to frame 0 :

![DH total transform 1][img_total_transform_1]

![DH total transform 2][img_total_transform_2]

![DH total transform 3][img_total_transform_3]

![DH total transform 4][img_total_transform_4]

![DH total transform 5][img_total_transform_5]

To go from this last frame to the gripper frame we have to compensate with the following transformation matrix :

![DH eef compensation][img_tf_eef_compensation]

By compensating the last frame we get the following transformation for the gripper frame respect to the base frame :

![img_tf_eef_base][img_tf_eef_base]

### 3. Inverse Kinematic Analysis

#### IK problem Decoupling

To solve the Inverse Kinematics of the manipulator we will take advantage of the fact that the 3 last joints have axes that met in a single point, which lets us decouple the inverse kinematics into two subproblems : 

*   **Find the first 3 joints** by decoupling the wrist and solving the position of the wrist ( which is define by the 3 first joints ).
*   **Find the last 3 joints** by solving for the orientation of the wrist ( which is defined by all joints, but as we already know the first 3, we only have to solve for the remaining 3 )

The following picture shows the wrist center, from which we will do the decoupling.

![img_ik_problem_separation][img_ik_problem_separation]

The information we are given is the required pose of the end effector, which consists of its position and orientation, information we will use to compute the wrist pose ( position and orientation ).

Recall the generalized total transformation between the gripper link and the base link :

![total ee transform][img_tf_eef_base]

From the last column we can get that the position of the end effector is computed from the position of the wrist center compensated with the gripper offset ( distance and orientation )

![IK decoupling step 1][img_ik_decoupling_1]

By solving for the wrist center, we get the following :

![IK decoupling step 2][img_ik_decoupling_2]

We see that we only need to have the orientation of the last frame in order to compute the wrist center, which can be done by using the orientation of the end effector and the compensation of the end effector respect to the 6th dh frame, as follows :

![IK decoupling step 3][img_ik_decoupling_3]

Once we have the wrist position and orientation, we can use the fact that these only depend on the first 3 joint variables, which will help us find these variables geometrically.
Then we can use them to compute the other 3 joint variables, which in conjunction with the first 3 joints give the orientation of the wrist. By replacing the solved joints into the rotation matrix for the first 3 frames, we can compute the rotation matrix for the last 3 frames, and then solve for the last 3 joints we need.

The whole process can be implemented in the following algorithm :

![IK solver algorithm][img_ik_solver_algorithm]

Almost all the necessary computations are explained in the algorithm above, except for the geometrical part that is used to solve for joints 2 and 3, which we will explain in the following lines.

#### IK geometrical solution for joints 2 and 3

The remaining joints 2 and 3 can be computed geometrically from the following figure :

![IK solution for joints 2 and 3 - case 1][img_ik_geometricSolution_case1]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]

