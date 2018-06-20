
#pragma once

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
        // happened, in order to make the correct ...
        // transition )
        int m_transitionCode;

        // World reference
        RWorld* m_world;

        public :

        RStateInterface( RWorld* pWorld )
        {
            m_world = pWorld;
        }

        ~RStateInterface() 
        {
            m_world = NULL;
        }

        RState id() { return m_stateId; }
        RStateMode mode() { return m_mode; }
        int transitionCode() { return m_transitionCode; }

        // State's entry point
        virtual void enter( int action ) = 0;
        // State's loop logic
        virtual void update( float dt ) = 0;
    };



}