
#include <kuka_arm/RDemoStateStart.h>

namespace kuka
{


    RDemoStateStart::RDemoStateStart( RWorld* pWorld )
        : RStateInterface( pWorld )
    {
        m_timer = 0.0;
        m_startWaitTime = 3.0;// Wait for 3 seconds
    }

    void RDemoStateStart::enter( int action )
    {
        m_timer = 0;
        m_mode = MODE_RUNNING;

        if ( action == RAction::INIT )
        {
            // Say hi
            m_world->ptrVisTools()->deleteAllMarkers();
            m_world->ptrVisTools()->loadRemoteControl();
            utils::showMessage( m_world->ptrVisTools(),
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
            m_world->moveGroup().setStartStateToCurrentState();
            m_world->moveGroup().setMaxVelocityScalingFactor( 0.2 );
            // Set up eef group with some initial values
            m_world->eefGroup().setMaxScalingFactor( 1.0 );
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