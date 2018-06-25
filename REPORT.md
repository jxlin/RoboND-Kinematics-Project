
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
[img_ik_geometricSolution_case2]: _imgs/img_ik_geometricSolution_case2.png
[img_ik_joints_1_2_3_solution]: _imgs/img_ik_joints_1_2_3_solution.png
[img_ik_solution_helpers]: _imgs/img_ik_solution_helpers.png

[img_ik_inverse_method_pt1]: _imgs/img_ik_inverse_method_pt1.png
[img_ik_inverse_method_pt2]: _imgs/img_ik_inverse_method_pt2.png
[img_ik_inverse_method_pt3]: _imgs/img_ik_inverse_method_pt3.png

[gif_fk_test]: _imgs/gif_fk_test.gif
[gif_ik_test_single_pose]: _imgs/gif_ik_test_single_pose.gif
[gif_ik_test_trajectory_dropoff]: _imgs/gif_ik_test_trajectory_dropoff.gif

[img_ik_publisher]: _imgs/img_ik_publisher.png
[img_trajectory_logging]: _imgs/img_trajectory_logging.png

[gif_pick_place_run]: _imgs/gif_pick_place_run.gif

[img_check_replan]: _imgs/img_check_replan.png
[img_user_extra_options]: _imgs/img_user_extra_options.png

[gif_leoJS_demo]: _imgs/gif_leoJS_demo.gif
[gif_leoJS_playground_demo]: _imgs/gif_leoJS_playground_demo.gif
[gif_leoJS_playground_editing]: _imgs/gif_leoJS_playground_editing.gif

[**Video of the submission**](http://www.youtube.com/watch?v=X3oHxq_AeLk)

## **About the project**

This project consists of implementing the Inverse Kinematics of the KUKA KR210 manipulator, in order to perform a pick and place task in a simulated environment.

![PICK-PLACE run][gif_pick_place_run]

The implementation is mainly in ROS, using the following tools :

*   **Gazebo**, for simulation of the pick and place task.
*   **RViz**, to visualize the kinematics of the manipulator.
*   **MoveIt**, to make planning actions and get the trajectories we must follow.

The work done in this project consists on doing the **Kinematic Analysis** of the Kuka KR210 manipulator in order to implement the **IK_server** ROS node, which will be in charge of giving a service for the environment to request joint trajectories, given the pose trajectories returned by the planner.

As a bonus I implemented a web tool in typescript which helped me in this analysis, as well as to understand some of the low level details that you run into when trying to implement something from scratch.

![leojs_demo_scene][img_leojs_demo_scene]

[Demo of pick-place in webtool](https://wpumacay.github.io/leoJS/index.html)

![leojs_demo_playground][img_leojs_demo_playground]

[Demo of dh maker in webtool](https://wpumacay.github.io/leoJS/playground.html)

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

From the last column we that the position of the end effector is computed from the position of the wrist center and the gripper offset ( distance and orientation )

![IK decoupling step 1][img_ik_decoupling_1]

By solving for the wrist center, we get the following :

![IK decoupling step 2][img_ik_decoupling_2]

We see that we only need to have the orientation of the last frame in order to compute the wrist center, which can be done by using the orientation of the end effector and the compensation of the end effector respect to the 6th dh frame, as follows :

![IK decoupling step 3][img_ik_decoupling_3]

Once we have the wrist position and orientation, we can use the fact that the first only depends on the first 3 joint variables, which will help us find these variables geometrically.
Then we can use them to compute the other 3 joint variables, which in conjunction with the first 3 joints give the orientation of the wrist. By replacing the solved joints into the rotation matrix for the first 3 frames, we can compute the rotation matrix for the last 3 frames, and then solve for the last 3 joints we need.

The whole process can be implemented in the following algorithm :

![IK solver algorithm][img_ik_solver_algorithm]

Almost all the necessary computations are explained in the algorithm above, except for the geometrical part that is used to solve for joints 2 and 3, which we will explain in the following lines.

#### IK geometrical solution for joints 2 and 3

The remaining joints 2 and 3 can be computed geometrically from the following figure :

![IK solution for joints 2 and 3 - case 1][img_ik_geometricSolution_case1]

![IK solution for joints 2 and 3 - case 2][img_ik_geometricSolution_case2]

There are two cases to consider because of the position of the wrist, and they yield the following equations for the remaining joints :

![IK solution joints 1, 2 and 3 equations][img_ik_joints_1_2_3_solution]

In this case only joint 2 has two separate cases, depending on the results of an intermediate angle.

All intermediate quantities are computed in the same manner for both cases ( the math for these intermediate quantities turns out to be the same for both cases ), and yield the following computations:

![IK solution 1,2,3 helpers][img_ik_solution_helpers]

Once we have the first three joint values, the remaining ones can be computed by solving for the wrist orientation, which is explained in the full algorithm shown above.

This concludes the calculations needed for the inverse kinematics of the manipulator, leaving only the implementation to be explained, which I proceed to explain in the following section

## **Project Implementation**

In the following subsections I will explain some of the details of my implementation, which in order to make it modular and reusable I had to separate some of the functionality into modules.

The first implementation was made in Typescript, because for some time I wanted to finish a basic robot visualizer in the browser.

After the implementation was made in Typescript, and the implementation worked correctly in FK and IK tasks, I just ported it to Python without some of the unnecessary parts, thus completing the project requirements after successfully picking the target and dropping it to the bin.

While porting I had to make some tools to check that my implementation was correct, because I didn't have access to a debugger, as in the Typescript implementation. This yield some test UI, some testing nodes and some extra launch files all in ROS, to check that the FK and IK implementations were correct.

### 0. DH library
    
The DH representation allows us to easily modularize each DH frame by abstracting away the functionality needed from the DH table, which in our case yield to the abstraction of the DH table entries and the DH model itself. 

First, the necessary data to hold into the abstraction of each DH entry in the table would be:

*   The parameters from it's entry in the DH table ( including joint offsets and sign, max-min ranges, joint type )
*   The current joint value
*   The transformation matrix itself ( numeric, with methods to update it from the joint value )
*   The transformation matrix itself ( symbolic, just for double checking )

With this in mind, the file [**RDHentry.py**](https://github.com/wpumacay/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/dhlibrary/RDHentry.py) contains the implementation of the abstraction of each entry in the DH table, just needing a mechanism to hold all these entries, which is the abstraction of the DH model itself.

This abstraction should have the following pieces: 

*   The total transform from frame 0 to frame 6.
*   The compensation matrix for the end effector.
*   The total transform from frame 0 to the end effector.
*   Getters and setters for handling the inner data.
*   Some utility functions.
*   A method to override in order to implement inverse kinematics for different model.

The implementation of the above abstraction can be found in the file [**RDHmodel.py**](https://github.com/wpumacay/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/dhlibrary/RDHmodel.py), which is the interface for the different types of manipulators for which we want to make its DH representation, leaving to the user only the implementation of the **_buildModel** method ( create DH entries, and defining the end effector compensation ) and the **inverse** kinematics method when deriving from this class.

### 1. Inverse kinematics implementation

Building from the previous abstractions, we extended the base DH model in order to implement the inverse kinematics for our Kuka KR210 manipulator, which can be found in the file [**RDHmodelKukaKR210.py**](https://github.com/wpumacay/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/dhlibrary/RDHmodelKukaKR210.py).

This file implements a child class for the base **RDHmodel** class, adding the functionality for the IK solver shown in the previous sections into the **inverse** method, which is shown below:

![IK inverse method pt1][img_ik_inverse_method_pt1]
![IK inverse method pt2][img_ik_inverse_method_pt2]
![IK inverse method pt3][img_ik_inverse_method_pt3]

### 2. Some useful tools

First, I had to test that the Forward Kinematics were correctly, and because RViz showed only the movement achieved by the kinematic tree, I had to compare the output of the end effector pose in RViz and the pose returned by my FK implementation.

To do this I made a node called [**FK_tester.py**](https://github.com/wpumacay/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/FK_tester.py) node a bit in order to publish the end effector poses from both RViz and the my **RDHmodel** implementation.

To run the test just execute the **FK_test.launch** file :

    roslaunch kuka_arm FK_test.launch

The results are shown in the following gif, were you can see in the plot that the poses are the same ( except for some delay between the time the poses are measured ):

![FK tester][gif_fk_test]

Then, I had to check that the IK implementation was correct, so I wanted to check that visually, as in the webtool I made ( will explain in the next sections ).

I ended up implementing a node called [**IK_tester.py**](https://github.com/wpumacay/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/IK_tester.py) to make this possible, basically sending the joints previously sent by the **joints_state_publisher** from previous tests, but this time these joints were calculated by the IK implementation in the model previous discussed.

I also wanted a way to manipulate the end effector pose, as I could in the FK test, by means of the joints_state_publisher UI, so I implemented a simple UI in the [**RIKpublisherUI.py**](https://github.com/wpumacay/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/utils/RIKpublisherUI.py), which allowed me to control just that.

![IK publisher][img_ik_publisher]

Finally, I added some functionality to check that my implementation was following the trajectories correctly, so I made a basic logger in the **trajectory_sampler.cpp** file and logged a single trajectory from the pick and place loop, which then I could sent using the UI by clicking a button.

![IK trajectory logging][img_trajectory_logging]

And here is how the tools work, in single pose mode, and trajectory mode :

![IK testing single pose][gif_ik_test_single_pose]

![IK testing trajectory dropoff][gif_ik_test_trajectory_dropoff]

### 3. Results

With the IK implementation discussed earlier I got to successfully complete the required number of picks and places of the project. One of these pick-place operations is shown below.

![PICK-PLACE run 2][gif_pick_place_run]

In the first launches some of the times the cycle crashed because of the planner, as a few times it didn't come up with a plan ( it just returned empty ). The logs in the **trajectory_sampler.cpp** helped to identify this issue, but when rvit happened that cycle failed spectacularly, and I had to press next and wait till the nexrvt cycle to continue the process. ( Btw, I was testing all these not in the VM, as it run vervry very slow ).
rv
To avoid this issue I added some checks in the **trajectory_samplerrv.cpp** in order to continue replan if the planner returned failure.rv

![check replan][img_check_replan]

Also, it was quite annoying to have to press next for every state in the pick and place cycle, so I just added a check in the **trajectory_sampler.cpp** to skip the prompting for the user if wanted.
This worked, but for some reason when grasping the object, sometimes this slipped from the gripper, so I avoid using this functionality in the submission. It seems that a small time has to be given to allow the gripper to grasp the object successfully, even when the event grasp returned successful ( at least that's what the log said ), and that time was the time that the user waited till pressed the next option in RViz.

All these configurations can be changed in the inverse_kinematics.launch file :

![checking options][img_user_extra_options]

And for a last test, just before running the IK implementation in the actual pick-place cycle, I ran the IK_debug.py script ( moved it also, to the scripts folder ) and this helped me find the second case I had not considered in the IK implementation, as it fail for a some test cases and then in the IK_tester as well.

After fixing these issues, everything worked correctly, just returning a different set of joint angles for case 2 as the IKsolver might yield a different but still correct solution. The wrist center and other checkings passed normally in for all test cases.

### 4. About the webtool

For some time I wanted to implement a robot visualizer in the browser, but I kept leaving it for later. The project gave me the perfect excuse to implement some of this stuff, as I was also implementing a rendering engine from scratch, to gain a bit more experience designing and implementing a bit bigger systems, instead of using already-made tools that kind of leave me frustrated as to how things worked under the hood. Also, I would have to learn how to use a special library ( like ThreeJS, or BabylonJS ) in order to build my visualizer, but as I saw in some of my friends projects, they spent more time learning how to hack into the library and make some specific stuff work, than making the actual implementation of what they wanted.

By no means I intend to say that you should always make things from scratch, but when learning, it might be useful to get your hands dirtier than required.

The [**visualizer**](https://github.com/wpumacay/leoJS) I implemented was based in Typescript and WebGL, and I built it on top of the [**engine**](https://github.com/wpumacay/cat1js) I was making for some other simulators-visualizer that I will make in the future.

[**Here**](https://wpumacay.github.io/leoJS/index.html) is the first demo, which consists of the scene of the pick and place project in the browser ( give it some time to load, as has to parse the urdfs and the collada files, and initialize some stuff ) :

![leojs demo][gif_leoJS_demo]

[**Here**](https://wpumacay.github.io/leoJS/playground.html) is the other demo, which is an editor that allows you to build a DH representation for a manipulator and play with it :

![leojs demo playground][gif_leoJS_playground_demo]

![leojs editing playground][gif_leoJS_playground_editing]

The editing part generates the DH representation that you request, and also creates the model if there is one ( like the KukaKR210 case, for which we have the urdf files ).