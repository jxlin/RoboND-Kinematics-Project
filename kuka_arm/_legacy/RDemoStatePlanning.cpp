
#include <kuka_arm/RDemoStatePlanning.h>




namespace kuka
{


    RDemoStatePlanning::RDemoStatePlanning( RWorld* pWorld )
        : RStateInterface( pWorld )
    {
        m_timer = 0;
        m_safeWaitTime = 0.5;
        m_lastAction = -1;
    }

    void RDemoStatePlanning::enter( int action )
    {
        m_lastAction = action;
        m_timer = 0;
        m_mode = MODE_RUNNING;

        if ( action == RAction::MAKE_PLAN_TO_TARGET )
        {
            std::cout << "RDemoStatePlanning> making plan to target" << std::endl;
            ROS_DEBUG( "RDemoStatePlanning> making plan to target" );

            utils::showMessage( m_world->ptrVisTools(),
                                "State> Planning path to target location",
                                rviz_visual_tools::WHITE,
                                rviz_visual_tools::XXXXLARGE );

            _makePlanToPose( m_world->targetPose() );
        }
        else if ( action == RAction::MAKE_PLAN_TO_RELEASE )
        {
            std::cout << "RDemoStatePlanning> making plan to bin" << std::endl;
            ROS_DEBUG( "RDemoStatePlanning> making plant to bin" );

            utils::showMessage( m_world->ptrVisTools(),
                                "State> Planning path to release location",
                                rviz_visual_tools::WHITE,
                                rviz_visual_tools::XXXXLARGE );

            _makePlanToPose( m_world->binPose() );
        }
        else if ( action == RAction::REPLAN )
        {
            std::cout << "RDemoStatePlanning> replanning..." << std::endl;
            ROS_DEBUG( "RDemoStatePlanning> replanning" );

            utils::showMessage( m_world->ptrVisTools(),
                                "State> Replanning ...",
                                rviz_visual_tools::WHITE,
                                rviz_visual_tools::XXXXLARGE );

            if ( m_lastAction == RAction::MAKE_PLAN_TO_TARGET )
            {
                _makePlanToPose( m_world.targetPose() );
            }
            else if ( m_lastAction == RAction::MAKE_PLAN_TO_RELEASE )
            {
                _makePlanToPose( m_world.binPose() );
            }
            else
            {
                std::cout << "RDemoStatePlanning> wtf???" << std::endl;
                ROS_DEBUG( "RDemoStatePlanning> wtf???" );
            }
        }
        else
        {
            std::cout << "RDemoStatePlanning> wtf???" << std::endl;
            ROS_DEBUG( "RDemoStatePlanning> wtf???" );
        }
    }

    void RDemoStatePlanning::update( float dt )
    {
        if ( m_mode != MODE_RUNNING )
        {
            return;
        }

        m_timer += dt;
        if ( m_timer >= m_safeWaitTime )
        {
            m_timer = 0;
            m_mode = MODE_FINISHED;
        }
    }


    void RDemoStatePlanning::_makePlanToPose( const geometry_msgs::Pose& pose )
    {
        moveit::planning_interface::MoveGroupInterface::Plan _plan;

        // Run planner and check if everything went fine
        if ( m_world->moveGroup().plan( m_world->plan() ) == 
             moveit::planning_interface::MoveItErrorCode::SUCCESS )
        {
            // Good, just publish some useful stuff
            m_world->ptrVisTools()->publishTrajectoryLine( m_world->plan().trajectory_,
                                                           m_world->ptrJointModelGroup() );
            m_mVisTools->trigger();

            m_transitionCode = RAction::EXECUTE_PLAN_TO_RELEASE;
        }
        else
        {
            std::cout << "RDemoStatePlanning> could not find a "
                      << "path to the requested location" << std::endl;
            ROS_INFO( "Could not find a path to the requested location. Replanning" );

            m_transitionCode = RAction::REPLAN;
        }
    }
}