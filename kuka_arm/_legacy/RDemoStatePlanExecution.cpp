
#include <kuka_arm/RDemoStatePlanExecution.h>



namespace kuka
{

    RDemoStatePlanExecution::RDemoStatePlanExecution()
    {
        m_lastAction = -1;
        m_timer = 0;
        m_safeWaitTime = 0.25;// Wait 0.25 seconds

        m_isDemo = true;
        m_logToFile = false;
        m_finishedTask = false;
    }

    RDemoStatePlanExecution::~RDemoStatePlanExecution()
    {

    }

    void RDemoStatePlanExecution::enter( int action )
    {
        m_timer = 0;
        m_lastAction = action;
        m_finishedTask = false;
        m_mode = MODE_RUNNING;

        m_path.clear();
        m_robotJointPositions.clear();

        if ( action == RAction::EXECUTE_PLAN_TO_TARGET ||
             action == RAction::EXECUTE_PLAN_TO_RELEASE )
        {
            bool _reachedTarget = false;
            if ( m_isDemo )
            {
                _reachedTarget = _useMoveitComponentForExecution();
            }
            else
            {
                _reachedTarget = _useIKserverComponentForExecution();
            }

            if ( _reachedTarget )
            {
                // Show current state message in rviz
                utils::showMessage( m_world->ptrVisTools(),
                                    "State> Plan execution - Reached target location",
                                    rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XXXXLARGE );
            }
            else
            {
                // Show current state message in rviz
                utils::showMessage( m_world->ptrVisTools(),
                                    "State> Plan execution - Error while moving to target location",
                                    rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XXXXLARGE );
            }

            m_finishedTask = true;
        }
    }

    void RDemoStatePlanExecution::update( float dt )
    {
        if ( m_mode != MODE_RUNNING )
        {
            return;
        }

        m_timer += dt;
        if ( m_timer >= m_safeWaitTime )
        {
            if ( m_finishedTask )
            {
                m_timer = 0;
                m_mode = MODE_FINISHED;
            }
        }
    }

    bool RDemoStatePlanExecution::_useMoveitComponentForExecution()
    {
        if ( m_logToFile )
        {
            _buildPosesRequest();
            tools::savePathToFile( "f_path_to_target.txt", m_path );
        }

        utils::showMessage( m_world->ptrVisTools(),
                            "State> Plan execution - moving to target location using moveit",
                            rviz_visual_tools::WHITE,
                            rviz_visual_tools::XXXXLARGE );

        bool _success = m_world->moveGroup().execute( m_world->plan() ) ==
                            moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if ( _success )
        {
            ROS_INFO( "RDemoStatePlanExecution> moveit execution - ok!" );
        }
        else
        {
            ROS_INFO( "RDemoStatePlanExecution> moveit execution - error :/" );
        }

        return _success;
    }

    bool RDemoStatePlanExecution::_useIKserverComponentForExecution()
    {
        _buildPosesRequest();

        // Some logging info
        {
            ROS_DEBUG_STREAM( "Total poses in path: " << m_world->plan().size()
                              << "Actual points in plan: "
                              << m_world->plan().trajectory_.joint_trajectory.points.size() );
            ROS_DEBUG_STREAM( "point1: " << m_path[0] << "point 2: " << m_path.back() );
        }

        // Show current state message in rviz
        utils::showMessage( m_world->ptrVisTools(),
                            "State> Plan execution - calculating Inverse Kinematics",
                            rviz_visual_tools::WHITE,
                            rviz_visual_tools::XXXXLARGE );

        // Send request to IK server
        kuka_arm::CalculateIK _ikSrvRequest;
        _ikSrvRequest.request.poses = m_path;
        _requestIKfromServer( _ikSrvRequest );

        // Show current state message in rviz
        utils::showMessage( m_world->ptrVisTools(),
                            "State> Plan execution - moving to target location using IK",
                            rviz_visual_tools::WHITE,
                            rviz_visual_tools::XXXXLARGE );

        // Make the robot move using the request's response
        bool _ok = true;
        for ( std::size_t q = 0; q < _ikSrvRequest.response.points.size(); q++ )
        {
            // Fill the joint values for this jointState
            for ( std::size_t j = 0; j < _ikSrvRequest.response.points[q].positions.size(); j++ )
            {
                m_robotJointPositions[j] = _ikSrvRequest.response.points[q].positions[j];
            }
            // Set this joint values as reference to the controller
            m_world->moveGroup().setJointValueTarget( m_robotJointPositions );
            // Make the controller go to the requested reference
            if ( m_world->moveGroup().move() )
            {
                ROS_INFO( "RDemoStatePlanExecution> Robot actuation successful :)" );
            }
            else
            {
                ROS_ERROR( "RDemoStatePlanExecution> Robot actuation unsuccessful :/" );
                _ok = false;
                break;
            }
        }

        return _ok;
    }

    void RDemoStatePlanExecution::_buildPosesRequest()
    {
        // Get current successful plan from world reference
        auto _plan = m_world->plan();
        // Set the trajectory to the current plan's trajectory
        m_world->ptrRobotTrajectory()->setRobotTrajectoryMsg( *( m_world->ptrRobotKinematicState() ),
                                                              m_world->plan().trajectory_ );

        for ( std::size_t q = 0; q < m_world->ptrRobotTrajectory()->getWayPointCount(); q++ )
        {
            const Eigen::Affine3d& _eefPose = 
                m_world->ptrRobotTrajectory()->getWayPoint( q ).getGlobalLinkTransform( 
                                                                    m_world->ptrJointModelGroup( "link_6" )
                                                                );
            geometry_msgs::Pose _gripperPose;
            tf::poseEigenToMsg( _eefPose, _gripperPose );
            m_path.push_back( _gripperPose );
        }
    }

    void RDemoStatePlanExecution::_requestIKfromServer( kuka_arm::CalculateIK& ikSrvRequest )
    {
        if ( m_world->ikService().call( ikSrvRequest ) )
        {
            ROS_DEBUG_STREAM( "RDemoStatePlaneExecution> Current Pose: " << ikSrvRequest.response );
        }
        else
        {
            ROS_ERROR( "RDemoStatePlanExecution> Failed to call service calculate_ik" );
        }

        // Check the consistency of the data ( same joints size )
        {
            auto _robotCurrentState = m_world->moveGroup().getCurrentState();
            _robotCurrentState->copyJointGroupPositions( m_world->ptrJointModelGroup(),
                                                         m_robotJointPositions );
            ROS_DEBUG( "RDemoStatePlanExecution> Total joints in robotJointPositions: %zd %zd",
                       m_robotJointPositions.size(), ikSrvRequest.response.points[0].positions.size() )
        }
    }
}