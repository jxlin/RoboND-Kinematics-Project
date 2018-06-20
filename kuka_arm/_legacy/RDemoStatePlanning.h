
#pragma once

#include <kuka_arm/RStateInterface.h>
#include <kuka_arm/RDemoCommon.h>

namespace kuka
{


    class RDemoStatePlanning : public RStateInterface
    {

        private :

        int m_lastAction;
        float m_timer;
        float m_safeWaitTime;// Wait a little bit before exiting the state

        void _makePlanToPose( const geometry_msgs::Pose& pose );

        public :

        RDemoStatePlanning( RWorld* pWorld );
        ~RDemoStatePlanning();

        void enter( int action ) override;
        void update() override;

    };



}