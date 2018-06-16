
#pragma once

#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>

#include "RWorld.h"

namespace kuka
{

    enum RState
    {
        STATE_START = 0,
        STATE_PLANNING = 1,
        STATE_PLAN_EXECUTION = 2,
        STATE_TARGET_GRASPING = 3,
        STATE_TARGET_RETRIEVING = 4,
        STATE_TARGET_RELEASING = 5
    };

    enum RStateMode
    {
        MODE_IDLE = 0,
        MODE_RUNNING = 1,
        MODE_FINISHED = 2
    };

    class RStateInterface
    {

        protected :

        // State's identifier
        RState m_stateId;
        // State's current execution mode
        RStateMode m_mode;
        // State's transition message ( used ...
        // by the state machine to decide what ...
        // happened in order to make the correct ...
        // transition )
        int m_transitionCode;

        // Resources - world reference
        RWorld* m_world;
        // Resources - MoveIt visual tools
        moveit_visual_tools::MoveItVisualTools* m_mVisTools;
        // Resources - MoveIt move group
        moveit::planning_interface::MoveGroupInterface m_moveGroup;
        // Resources - MoveIt eef group
        moveit::planning_interface::MoveGroupInterface m_eefGroup;
        // Resources - joint model group
        robot_state::JointModelGroup* m_jointModelGroup;
        // Resources - gripper model group
        robot_state::JointModelGroup* m_gripperModelGroup;

        public :

        RStateInterface( RWorld* pWorld,
                         moveit_visual_tools::MoveItVisualTools* pVisTools,
                         moveit::planning_interface::MoveGroupInterface* pMoveGroup,
                         moveit::planning_interface::MoveGroupInterface* pEefGroup,
                         robot_state::JointModelGroup* pJointModelGroup,
                         robot_state::JointModelGroup* pGripperModelGroup )
        {
            m_world = pWorld;

            m_mVisTools = pVisTools;

            m_moveGroup = pMoveGroup;
            m_eefGroup = pEefGroup;

            m_jointModelGroup = pJointModelGroup;
            m_gripperModelGroup = pGripperModelGroup;
        }

        ~RStateInterface() 
        {
            m_mVisTools = NULL;
        }

        RState id() { return m_stateId; }
        RStateMode mode() { return m_mode; }
        int transitionMsg() { return m_transitionCode; }

        // State's entry point
        virtual void enter( int action ) = 0;
        // State's loop logic
        virtual void update( float dt ) = 0;
    };



}