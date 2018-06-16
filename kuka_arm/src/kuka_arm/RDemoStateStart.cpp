
#include <kuka_arm/RDemoStateStart.h>

namespace kuka
{


    RDemoStateStart::RDemoStateStart( RWorld* pWorld,
                                      moveit_visual_tools::MoveItVisualTools* pVisTools,
                                      moveit::planning_interface::MoveGroupInterface* pMoveGroup,
                                      moveit::planning_interface::MoveGroupInterface* pEefGroup,
                                      robot_state::JointModelGroup* pJointModelGroup,
                                      robot_state::JointModelGroup* pGripperModelGroup )
        : RStateInterface( pWorld, pVisTools, pMoveGroup, pEefGroup, pJointModelGroup, pGripperModelGroup )
    {
        m_startWaitTime = 3.0;// Wait for 3 seconds
    }

    void RDemoStateStart::enter( int action )
    {
        if ( action == RAction::INIT )
        {
            // Say hi
            m_mVisTools->deleteAllMarkers();
            m_mVisTools->loadRemoteControl();
            utils::showMessage( m_mVisTools,
                                "State> Start of cycle",
                                rviz_visual_tools::WHITE,
                                rviz_visual_tools::XXXXLARGE );


            // Get target and release poses ...
            // from parameter server
            float _targetX, _targetY, _targetZ;
            float _binX, _binY, _binZ;
            ros::param::get( "/target_spawn_location/x", _targetX );
            ros::param::get( "/target_spawn_location/y", _targetY );
            ros::param::get( "/target_spawn_location/z", _targetZ );
            ros::param::get( "/target_drop_location/x", _binX );
            ros::param::get( "/target_drop_location/y", _binY );
            ros::param::get( "/target_drop_location/z", _binZ );

            m_world->targetPose().position.x = _targetX - 0.4;
            m_world->targetPose().position.y = _targetY;
            m_world->targetPose().position.z = _targetZ - 0.1;

            m_world->binPose().position.x = _binX - 0.1;
            m_world->binPose().position.y = _binY;
            m_world->binPose().position.z = _binZ + 1.6;

            // Set up move group with some initial values
            m_moveGroup->setStartStateToCurrentState();
            m_moveGroup->setMaxVelocityScalingFactor( 0.2 );
            // Set up eef group with some initial values
            m_eefGroup->setMaxScalingFactor( 1.0 );

            m_timer = 0;
            m_mode = MODE_RUNNING;
        }
    }

    void RDemoStateStart::update( float dt )
    {
        if ( m_mode != MODE_RUNNING )
        {
            // Just in case
            return;
        }

        m_timer += dt;
        if ( m_timer >= m_startWaitTime )
        {
            m_timer = 0;
            // Alright, it's time to begin the sequence
            m_transitionCode = RAction::MAKE_PLAN_TARGET;
            m_mode = MODE_FINISHED;
        }

    }

}