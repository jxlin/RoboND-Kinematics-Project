
#pragma once

#include <kuka_arm/RStateInterface.h>
#include <kuka_arm/RDemoCommon.h>

namespace kuka
{


    class RDemoStateStart : public RStateInterface
    {

        private :

        float m_timer;
        float m_startWaitTime;

        public :

        RDemoStateStart( RWorld* pWorld );
        ~RDemoStateStart();

        void enter( int action ) override;
        void update() override;

    };



}