/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Okke Hendriks
*    Date  : 2013/12/16
*         - File created.
*
* Description:
*   This is a node that listens to the status messages of the lift and publishes
*   transforms accordingly.
* 
***********************************************************************************/
#include "lift_controller/lift_tf.hpp"

TFLift::TFLift(string transform_name, ros::NodeHandle n, string from_link, string to_link)
    : TFHelper(transform_name, n, from_link, to_link)
{
    // Subscribe to lift status and register callback function on the lift status message topic
    lift_status_sub_ = n_.subscribe("lift_controller/lift/state", 50, &TFLift::CB_LiftStateChanged, this);
    ROS_INFO_NAMED(ROS_NAME, "'%s' has subscribed to 'lift_controller/lift/state'.", transform_name_.c_str());

    //! @todo MdL: Just to initialize for now.
    setLiftTransform(0);
}

TFLift::~TFLift()
{
}

void TFLift::CB_LiftStateChanged(const rose21_platform::lift_state::ConstPtr& lift_message)
{
    ROS_DEBUG_NAMED(ROS_NAME, "Lift position changed, updating transform: %d%%.", lift_message->position_percentage);

    setLiftTransform(lift_message->position_percentage);
}

void TFLift::setLiftTransform(int position_percentage)
{
    float tf_yaw    = 0.0;
    float tf_pitch  = 0.0;
    float tf_roll   = 0.0;
    float tf_x      = -(0.45-0.335)/2;
    // float tf_x      = -0.035;
    float tf_y      = 0.0;
    float tf_z      = 0.525;
    // float tf_z      = 0.543+0.024; // Bolts added at wheels (1,2cm)

    // ROS_DEBUG_NAMED(ROS_NAME, "Setting transform '%s'", transform_name_.c_str());
    // ROS_ERROR_NAMED(ROS_NAME, "Update this with a function for position percentage -> actual position.");
    this->setTransform(tf_roll, tf_pitch, tf_yaw, tf_x, tf_y, tf_z);
}
