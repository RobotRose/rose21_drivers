/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2013/12/10
*       - File created.
*
* Description:
*   The lift contoller node, contains the main function
* 
***********************************************************************************/

#include "rose21_lift_controller/lift_controller_node.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lift_controller");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    ros::Rate r(50);

    // Retrieve port and baudrate parameters
    string serial_port      = "";
    int baudrate            = 0;
    bool test               = false;
    if(!n_private.getParam("serial_port", serial_port))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Parameter serial_port must be specified.");
        return 1;
    }

    if(!n_private.getParam("baudrate", baudrate))
    {
        ROS_ERROR_NAMED(ROS_NAME, "Parameter baudrate must be specified.");
        return 1;
    }

    n_private.getParam("test", test);


    LiftController* lift_controller = new LiftController(ROS_NAME, n, serial_port, baudrate);

    bool    duurtest = false;
    bool    toggle   = true;
    int     timescnt = 0;

    while(n.ok())
    {
        if(rose_conversions::kbhit() && test)
        {
            uint c = getchar();
            ROS_DEBUG_NAMED(ROS_NAME, "Key pressed: %c", (char)c);
            switch(c)
            {
                case 'a':   lift_controller->setPose(100, 0);
                            break;
                case 'b':   lift_controller->setPose(100, 25);
                            break;
                case 'c':   lift_controller->setPose(100, 50);
                            break;
                case 'd':   lift_controller->setPose(100, 75);
                            break;
                case 'e':   lift_controller->setPose(100, 100);
                            break;
                case 's':   lift_controller->enable();
                            break;
                case 'q':   lift_controller->disable();
                            break;  
                case 'f':   lift_controller->forceMotorStop();
                            break;  
                case 'g':   lift_controller->resetAlarm();
                            break;
                case 'l':   duurtest = true;
                            break;
            }
        }

        if(duurtest)    //! @todo OH: REMOVE
        {
            
            ROS_INFO_NAMED(ROS_NAME, "Position: %d | Duurtest cnt: %d", lift_controller->getPose() , timescnt);
            if(toggle and lift_controller->getPose() >= 49)
            {
                lift_controller->setPose(50, 0);
                toggle = false;
                timescnt++;
            } 
            else if(lift_controller->getPose() <= 1)
            {
                 lift_controller->setPose(100, 50);
                 toggle = true;
            }
        }

        if(lift_controller->isEnabled())
        {
            // Check the watchdog
            if(!lift_controller->checkWatchdog())
            {
                ROS_WARN_NAMED(ROS_NAME, "Watchdog error, will try to reset communication.");
                if(!lift_controller->reset())
                    ros::Duration(1.0).sleep(); 
            }
            else
            {
                lift_controller->update();
                lift_controller->publishLiftState();
                lift_controller->publishBumpersState();
            }
        }
        else
        {
            if(lift_controller->getRequestedEnabledState())
                // Try to re-enable
                lift_controller->enable();
        }

        if(test)
            lift_controller->showState();

        ros::spinOnce();
        r.sleep();
    }

    delete lift_controller;
    
    return 0;
}

 