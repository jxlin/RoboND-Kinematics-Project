
#include <kuka_arm/RDemoStateMachine.h>



namespace kuka
{



    RDemoStateMachine::RDemoStateMachine()
    {
        // Initialize user flags
        m_waitForUserBetweenStates = false;
        m_waitForUserBetweenCycles = false;

        // Initialize states
        auto _stStart      = new RDemoStateStart();
        auto _stPlanning   = new RDemoStatePlanning();
        auto _stPlanExec   = new RDemoStatePlanExecution();
        auto _stGrasping   = new RDemoStateTargetGrasping();
        auto _stRetrieving = new RDemoStateTargetRetrieving();
        auto _stReleasing  = new RDemoStateTargetReleasing();

        m_states.push_back( _stStart );
        m_states.push_back( _stPlanning );
        m_states.push_back( _stPlanExec );
        m_states.push_back( _stGrasping );
        m_states.push_back( _stRetrieving );
        m_states.push_back( _stReleasing );

        m_currentState = NULL;
    }


    void RDemoStateMachine::enter()
    {
        m_currentState = m_states[ STATE_START ];
        m_currentState->enter( RAction::INIT );
    }

    void RDemoStateMachine::update( float dt )
    {
        if ( m_currentState == NULL )
        {
            std::cout << "RDemoStateMachine> there is " 
                      << "no current state :o" 
                      << std::endl;
            return;
        }

        if ( m_currentState->mode() == MODE_RUNNING )
        {
            // State still running, just let it run
            m_currentState->update( dt );
        }
        else if ( m_currentState->mode() == MODE_FINISHED )
        {
            bool _ok = false;
            std::string _transitionMsg = m_currentState->transitionMsg();

            // Make the appropiate transition
            if ( m_currentState->id() == STATE_START )
            {
                if ( _transitionMsg == RAction::MAKE_PLAN_TARGET )
                {
                    _ok = _makeTransition( STATE_PLANNING, 
                                           RAction::MAKE_PLAN_TO_TARGET );
                }
            }
            else if ( m_currentState->id() == STATE_PLANNING )
            {
                if ( _transitionMsg == RAction::EXECUTE_PLAN_TO_TARGET )
                {
                    _ok = _makeTransition( STATE_PLAN_EXECUTION, 
                                           RAction::EXECUTE_PLAN_TO_TARGET );
                }
                else if ( _transitionMsg == RAction::EXECUTE_PLAN_TO_RELEASE )
                {
                    _ok = _makeTransition( STATE_PLAN_EXECUTION,
                                           RAction::EXECUTE_PLAN_TO_RELEASE );
                }
                else if ( _transitionMsg == RAction::REPLAN )
                {
                    _ok = _makeTransition( STATE_PLANNING, 
                                           RAction::REPLAN );
                }
            }
            else if ( m_currentState->id() == STATE_PLAN_EXECUTION )
            {
                if ( _transitionMsg == RAction::GRASP_TARGET )
                {
                    _ok = _makeTransition( STATE_TARGET_GRASPING, 
                                           RAction::GRASP_TARGET );
                }
                else ( _transitionMsg == RAction::RELEASE_TARGET )
                {
                    _ok = _makeTransition( STATE_TARGET_RELEASING,
                                           RAction::RELEASE_TARGET );
                }
            }
            else if ( m_currentState->id() == STATE_TARGET_GRASPING )
            {
                if ( _transitionMsg == RAction::RETRIEVE_TARGET )
                {
                    _ok = _makeTransition( STATE_TARGET_RETRIEVING, 
                                           RAction::RETRIEVE_TARGET );
                }
            }
            else if ( m_currentState->id() == STATE_TARGET_RETRIEVING )
            {
                if ( _transitionMsg == RAction::MAKE_PLAN_TO_RELEASE )
                {
                    _ok = _makeTransition( STATE_PLANNING, 
                                           RAction::MAKE_PLAN_TO_RELEASE);
                }
            }
            else if ( m_currentState->id() == STATE_TARGET_RELEASING )
            {
                if ( _transitionMsg == RAction::INIT )
                {
                    _ok = _makeTransition( STATE_START, 
                                           RAction::INIT );
                }
            }

            if ( !_ok )
            {
                std::cout << "RDemoStateMachine> something went wrong "
                          << "while making a transition from state: "
                          << m_currentState->id() << " and transition msg: "
                          << _transitionMsg << std::endl;
            }
        }
        else if ( m_currentState->mode() == MODE_IDLE )
        {
            std::cout << "RDemoStateMachine> the current "
                      << "state is in idle. It should not be "
                      << "like that :x"
                      << std::endl;
        }
        
    }

    bool RDemoStateMachine::_makeTransition( RState newStateId )
    {
        if ( newStateId < STATE_START ||
             newStateId > STATE_TARGET_RELEASING )
        {
            std::cout << "RDemoStateMachine> requested "
                      << "new state: " + newStateId + " is not "
                      << "a valid state" << std::endl;
            return false;
        }

        m_currentState->exit();
        m_currentState = m_states[ newStateId ];
        m_currentState->enter();

        return true;
    }
}