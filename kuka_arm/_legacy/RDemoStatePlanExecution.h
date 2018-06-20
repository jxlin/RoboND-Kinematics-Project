
#pragma once

#include <kuka_arm/RStateInterface.h>
#include <kuka_arm/RDemoCommon.h>

namespace kuka
{


    class RDemoStatePlanExecution : public RStateInterface
    {

        private :

        int m_lastAction;
        float m_timer;
        float m_safeWaitTime;// Wait a little bit before exiting the state

        /**
        * m_isDemo = true; use moveit for IK
        *          = false; use IK_Server for IK
        */
        bool m_isDemo;
        /**
        * m_logToFile = true; log eef poses to a file
        *             = false; no logs
        */
        bool m_logToFile;

        bool m_finishedTask;

        std::vector< geometry_msgs::Pose > m_path;
        std::vector< double > m_robotJointPositions;

        bool _useMoveitComponentForExecution();
        bool _useUserIKserverComponentForExecution();

        void _buildPosesRequest();
        void _requestIKfromServer( kuka_arm::CalculateIK& ikSrvRequest );

        public :

        RDemoStatePlanExecution( RWorld* pWorld,
                                 moveit_visual_tools::MoveItVisualTools* pVisTools,
                                 moveit::planning_interface::MoveGroupInterface* pMoveGroup,
                                 moveit::planning_interface::MoveGroupInterface* pEefGroup,
                                 robot_state::JointModelGroup* pJointModelGroup,
                                 robot_state::JointModelGroup* pGripperModelGroup );
        ~RDemoStatePlanExecution();

        void enter( int action ) override;
        void update() override;

    };



}