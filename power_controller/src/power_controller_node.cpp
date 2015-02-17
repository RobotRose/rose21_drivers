/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/08/22
*       - File created.
*
* Description:
*   ROS driver power controller node
* 
***********************************************************************************/
#include <power_controller/power_controller_node.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "power_controller");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    ros::Rate r(10);

    // Retrieve port and baudrate parameters
    string serial_port  = "";
    int baudrate        = 0;
    bool test           = false;
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


    PowerController* power_controller = new PowerController("power_controller", n, serial_port, baudrate);

    if(!power_controller->enable())
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not enable power controller.");
        return 1;
    }

    while(n.ok())
    {
        if(rose_conversions::kbhit() && test)
        {
            uint c = getchar();
            ROS_DEBUG_NAMED(ROS_NAME, "Key pressed: %c", (char)c);
            switch(c)
            {
                case '1':
                    power_controller->togglePowerOutput(0);
                    break;
                case '2':
                    power_controller->togglePowerOutput(1);
                    break;
                case '3':
                    power_controller->togglePowerOutput(2);
                    break;
                case '4':
                    power_controller->togglePowerOutput(3);
                    break;
                case '5':
                    power_controller->togglePowerOutput(4);
                    break;
                case '6':
                    power_controller->togglePowerOutput(5);
                    break;
                case 'c':
                    power_controller->resetComm();
                    break;
                case 's':
                    power_controller->shutdown(10000, -1);
                    break;
                case 'd':   // Rebooot
                    power_controller->shutdown(10000, 15000);
                    break;
                case ',':
                    power_controller->setSwitchSound(true);
                    break;
                case '.':
                    power_controller->setSwitchSound(false);
                    break;
                case 'q':
                    power_controller->decreaseSoundInterval();
                    break;
                case 'w':
                    power_controller->increaseSoundInterval();
                    break;
                case 'o':
                    power_controller->setAlarmSound(1);
                    break;
                case 'p':
                    power_controller->setAlarmSound(0);
                    break;                    
                case 'a':
                    power_controller->setAutoSwitchMode(1);
                    break;  
                case 'm':
                    power_controller->setAutoSwitchMode(0);
                    break;   
                case 'b':
                    power_controller->selectFullBattery();
                    break;
                case '[':
                    power_controller->selectBattery(1);
                    break;
                case ']':
                    power_controller->selectBattery(2);
                    break;
                case '-':
                    power_controller->selectBattery(0);
                    break;
                case 'r':
                    power_controller->resetAllADCMinMax();
                    break;

            }
        }

        // Check the watchdog
        if(!power_controller->checkWatchdog())
        {
            ROS_WARN_NAMED(ROS_NAME, "Watchdog error, will try to reset communication.");
            if(!power_controller->resetComm())
                ros::Duration(1.0).sleep();
            else
                power_controller->setDefaults();
        }
        else 
        {
            power_controller->update();
            power_controller->publishBatteryStates();
            power_controller->showState();
        }

        ros::spinOnce();
        r.sleep();
    }

    delete power_controller;
    
    return 0;
}
