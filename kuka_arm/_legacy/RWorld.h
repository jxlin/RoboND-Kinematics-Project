
#pragma once

#include "RDemoCommon.h"

namespace kuka
{



    class RWorld
    {

        private :

        geometry_msgs::Pose m_binPose;
        geometry_msgs::Pose m_targetPose;
        // Resources - ros service for IK server
        ros::ServiceClient m_ikService;
        // Resources - MoveIt visual tools
        moveit_visual_tools::MoveItVisualTools* m_ptrVisTools;
        // Resources - MoveIt move group
        moveit::planning_interface::MoveGroupInterface m_moveGroup;
        // Resources - MoveIt eef group
        moveit::planning_interface::MoveGroupInterface m_eefGroup;
        // Resources - Moveit created plan
        moveit::planning_interface::MoveGroupInterface::Plan m_plan;
        // Resources - planning scene
        planning_scene::PlanningScene* m_ptrPlanningScene;
        // Resources - Robot state
        robot_state::RobotState* m_ptrRobotKinematicState;
        // Resources - Robot kinematic model
        robot_model::RobotModelPtr m_ptrRobotKinematicModel;
        // Resources - Robot trajectory ( just keep it global, as it's shared )
        robot_trajectory::RobotTrajectory* m_ptrRobotTrajectory;
        // Resources - joint model group
        robot_state::JointModelGroup* m_ptrJointModelGroup;
        // Resources - gripper model group
        robot_state::JointModelGroup* m_ptrGripperModelGroup;

        public :

        RWorld();
        ~RWorld();

        // TODO: Passing whole obj references is dangerous, should add const?
        // This object kind of acts as a big global variable, so maybe not.
        geometry_msgs::Pose& binPose() { return m_binPose; }
        geometry_msgs::Pose& targetPose() { return m_targetPose; }
        ros::ServiceClient& ikService() { return m_ikService;}
        moveit::planning_interface::MoveGroupInterface::Plan& plan() { return m_plan; }
        moveit::planning_interface::MoveGroupInterface& moveGroup() { return m_moveGroup; }
        moveit::planning_interface::MoveGroupInterface& eefGroup() { return m_eefGroup; }


        moveit_visual_tools::MoveItVisualTools* ptrVisTools() { return m_ptrVisTools; }
        robot_model::RobotModelPtr ptrRobotKinematicModel() { return m_ptrRobotKinematicModel; }
        robot_trajectory::RobotTrajectory* ptrRobotTrajectory() { return m_ptrRobotTrajectory; }
        robot_state::RobotState* ptrRobotKinematicState() { return m_ptrRobotKinematicState; }
        robot_state::JointModelGroup* ptrJointModelGroup() { return m_ptrJointModelGroup; }
        robot_state::JointModelGroup* ptrGripperModelGroup() { return m_ptrGripperModelGroup; }
    };





}