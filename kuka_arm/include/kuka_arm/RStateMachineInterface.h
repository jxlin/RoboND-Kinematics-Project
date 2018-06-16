
#pragma once

#include "RStateInterface.h"

#include <vector>



namespace kuka
{

    class RStateMachineInterface
    {

        protected :

        std::vector< RStateInterface* > m_states;
        RStateInterface* m_currentState;

        public :

        RStateMachineInterface() {}
        ~RStateMachineInterface() {}

        // StateMachine's entry point
        virtual void enter() = 0;
        // Update state machine here
        virtual void update( float dt ) = 0;

    };


}