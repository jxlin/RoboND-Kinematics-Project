

#pragma once

#include "RStateMachineInterface.h"

#include "RDemoStateStart.h"
#include "RDemoStatePlanning.h"
#include "RDemoStatePlanExecution.h"
#include "RDemoStateTargetGrasping.h"
#include "RDemoStateTargetRetrieving.h"
#include "RDemoStateTargetReleasing.h"

#include "RDemoCommon.h"

namespace kuka
{

    class RDemoStateMachine : public RStateMachineInterface
    {

        private :

        bool m_waitForUserBetweenStates;
        bool m_waitForUserBetweenCycles;

        // helper to make the transition between states
        bool _makeTransition( RState newStateId );

        public :

        RDemoStateMachine();
        ~RDemoStateMachine();

        void enter() override;
        void update( float dt ) override;


    };



}