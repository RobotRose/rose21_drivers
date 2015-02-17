/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2013/12/16
* 		- File created.
*
* Description:
*	This is a node that listens to the status messages of the lift and publishes
* 	transforms accordingly.
* 
***********************************************************************************/

#ifndef LIFT_TF_HPP
#define LIFT_TF_HPP

#include <stdio.h>
#include <string>

#include "tf_helper/tf_helper.hpp"
 
#include "rose21_platform/lift_state.h"

using std::string;

class TFLift : public TFHelper
{
  public:
    TFLift(string transform_name, ros::NodeHandle n, string from_link, string to_link);
    ~TFLift();

  private:
    void CB_LiftStateChanged(const rose21_platform::lift_state::ConstPtr& lift_message);
    void setLiftTransform(int pose);

    ros::Subscriber lift_status_sub_;
};


#endif // LIFT_TF_HPP
