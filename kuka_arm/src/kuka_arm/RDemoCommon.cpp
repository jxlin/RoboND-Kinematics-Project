
#include <kuka_arm/RDemoCommo.h>


namespace kuka
{


    namespace utils
    {


        void showMessage( moveit_visual_tools::MoveItVisualTools* pVisTools,
                          const std::string& msg,
                          const rviz_visual_tools::colors& msgColor,
                          const rviz_visual_tools::scales& msgScale )
        {
            // Pose to place the message
            Eigen::Affine3d _textPose = Eigen::Affine3d::Identity();
            _textPose.translation().z() = 4.0;

            // Use vizTools to publish text
            pVisTools->publishText( _textPose,
                                    msg.c_str(),
                                    msgColor,
                                    msgScale );
            pVisTools->trigger();

            
        }


    }

}