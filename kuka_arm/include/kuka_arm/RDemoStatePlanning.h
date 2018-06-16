
#pragma once

#include <kuka_arm/RStateInterface.h>
#include <kuka_arm/RDemoCommon.h>

namespace kuka
{


    class RDemoStatePlanning : public RStateInterface
    {

        private :

        int m_lastAction;
        float m_timer;
        float m_safeWaitTime;// Wait a little bit before exiting the state

        void _makePlanToPose( const geometry_msgs::Pose& pose );

        public :

        RDemoStatePlanning( RWorld* pWorld,
                            moveit_visual_tools::MoveItVisualTools* pVisTools,
                            moveit::planning_interface::MoveGroupInterface* pMoveGroup,
                            moveit::planning_interface::MoveGroupInterface* pEefGroup,
                            robot_state::JointModelGroup* pJointModelGroup,
                            robot_state::JointModelGroup* pGripperModelGroup );
        ~RDemoStatePlanning();

        void enter( int action ) override;
        void update() override;

    };



}