/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2013/12/16
*       - File created.
*
* Description:
*   The lift transform node
* 
***********************************************************************************/    

#include "lift_controller/lift_tf_node.hpp"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_lift");
    ros::NodeHandle n;
    ros::Rate rate(60);

    ROS_INFO_NAMED(ROS_NAME, "Lift transform node started");

    // Create transform object
    TFLift* tf_lift = new TFLift(ROS_NAME, n, "/base_link", "/lift_link");

    // Keep on spinnin
    while(n.ok())
    {
        tf_lift->Broadcast();
        ros::spinOnce();
        rate.sleep();
    }
    
    // Cleanup
    delete tf_lift;
        
    return 0;
}