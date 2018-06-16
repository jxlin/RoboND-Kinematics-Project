
#pragma once

#include <kuka_arm/RStateInterface.h>
#include <kuka_arm/RDemoCommon.h>

namespace kuka
{


    class RDemoStateStart : public RStateInterface
    {

        private :

        float m_timer;
        float m_startWaitTime;

        public :

        RDemoStateStart( RWorld* pWorld,
                         moveit_visual_tools::MoveItVisualTools* pVisTools,
                         moveit::planning_interface::MoveGroupInterface* pMoveGroup,
                         moveit::planning_interface::MoveGroupInterface* pEefGroup,
                         robot_state::JointModelGroup* pJointModelGroup,
                         robot_state::JointModelGroup* pGripperModelGroup );
        ~RDemoStateStart();

        void enter( int action ) override;
        void update() override;

    };



}