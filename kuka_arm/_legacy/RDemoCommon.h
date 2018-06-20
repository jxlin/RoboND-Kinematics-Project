
#pragma once

#include <string>
#include <iostream>
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

namespace kuka
{

    namespace RAction
    {
        enum _RAction
        {
            INIT,
            MAKE_PLAN_TARGET,
            EXECUTE_PLAN_TO_TARGET,
            EXECUTE_PLAN_TO_RELEASE,
            REPLAN,
            GRASP_TARGET,
            RETRIEVE_TARGET,
            MAKE_PLAN_TO_RELEASE,
            RELEASE_TARGET
        }
    }

    namespace utils
    {
        void showMessage( moveit_visual_tools::MoveItVisualTools* pVisTools,
                          const std::string& msg,
                          const rviz_visual_tools::colors& color,
                          const rviz_visual_tools::scales& scale );
    }
}