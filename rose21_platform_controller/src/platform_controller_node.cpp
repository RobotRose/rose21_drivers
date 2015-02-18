/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2013/12/17
* 		- File created.
*
* Description:
*	The platform_controller_node
* 
***********************************************************************************/

#include "rose21_platform_controller/platform_controller_node.hpp"

/**\defgroup platform_controller_node Platform controller node
 * \relates PlatformController
 * Main function of the platform_controller node. Create a PlatformController object and calls spinning code.
 * Use keys for simple manual control:
 *  \arg \b + Increase linear y-velocity
 *  \arg \b - Decrease linear y-velocity 
 *  \arg \b * Increase wheelunit rotation
 *  \arg \b / Decrease wheelunit rotation
 *  \arg \b s Stop all movement
 *  \arg \b x Stop all movement and quit
 * @param[in] Command line argurments
 * @return bool Exit state 
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "platform_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	ros::Rate r(100);			// A.t.m. limited by low-level controller to ~25hz

	// PID log files
	FILE * pid_log_drive = NULL;
	FILE * pid_log_steer = NULL;
	auto time_start_sec  = ros::Time::now().toSec();

	// Retrieve port and baudrate parameters
	string serial_port    = "";
    int baudrate          = 0;
    bool test 			  = false;
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
    if(test)
    	ROS_INFO_NAMED(ROS_NAME, "Test mode enabled.");

	PlatformController* platform_controller = new PlatformController("platform_controller", n, serial_port, baudrate);
	if(!platform_controller->enable())
	{
		ROS_ERROR_NAMED(ROS_NAME, "Could not enable platform controller.");
		return 1;
	}


	float 	velocity 			= 0.0;
	float 	orientation			= 0.0;
	float 	vel_step 			= (2.0*M_PI)/40.0;
	float 	orientation_step 	= M_PI/16.0;
	bool 	active_braking		= false;
	bool 	stop 				= false;
	bool 	log_PID 			= false;

	ros::Time begin;
	while(n.ok() && !stop)
	{
		begin = ros::Time::now();

		if(rose_conversions::kbhit() && test)
		{
		    uint c = getchar();
		    ROS_INFO_NAMED(ROS_NAME, "Key pressed: %c", (char)c);
		    switch(c)
		    {
		    	case '+':	
		    		velocity += vel_step;    		
					for(auto& wheelunit : platform_controller->getWheelUnits())
						wheelunit.setVelocityRadPerSec(velocity);

					break;
				case '-':	
		    		velocity -= vel_step;    		
					for(auto& wheelunit : platform_controller->getWheelUnits())
						wheelunit.setVelocityRadPerSec(velocity);

					break;
				case '*':
					orientation += orientation_step;
					ROS_INFO_NAMED(ROS_NAME, "Orientation: %.2f", orientation);
					for(auto& wheelunit : platform_controller->getWheelUnits())
						wheelunit.setAngleRad(orientation);	

					break;
				case '/':
					orientation -= orientation_step;
					ROS_INFO_NAMED(ROS_NAME, "Orientation: %.2f", orientation);
					for(auto& wheelunit : platform_controller->getWheelUnits())
						wheelunit.setAngleRad(orientation);

					break;
				case 'r':
					platform_controller->forceReset();		
					break;
				case 's':
				    velocity    = 0.0;
				    orientation = 0.0;
					for(auto& wheelunit : platform_controller->getWheelUnits())
					{
						wheelunit.setAngleRad(orientation);
						wheelunit.setVelocityRadPerSec(velocity);	
					}
					break;
				case '[':
					ROS_INFO_NAMED(ROS_NAME, "PID logging on");
					if(pid_log_drive == NULL)
						pid_log_drive = fopen("PID_data_drive.dat", "w+");	

					if(pid_log_steer == NULL)
						pid_log_steer = fopen("PID_data_steer.dat", "w+");

					if(pid_log_drive == NULL)
						ROS_WARN_NAMED(ROS_NAME, "Could not open PID drive log file.");			

					if(pid_log_steer == NULL)
						ROS_WARN_NAMED(ROS_NAME, "Could not open PID steer log file.");			

					log_PID = true;
					break;
				case ']':
					ROS_INFO_NAMED(ROS_NAME, "PID logging off");
					if(pid_log_drive != NULL  && pid_log_steer != NULL)
					{
						fclose(pid_log_drive);
						fclose(pid_log_steer);
						pid_log_drive = NULL;
						pid_log_steer = NULL;
					}

					log_PID = false;		
					break;
				case 'b':
					if(active_braking)
					{
						ROS_INFO_NAMED(ROS_NAME, "Passive brake drive mode.");
						active_braking = false;
					}
					else
					{
						ROS_INFO_NAMED(ROS_NAME, "Active brake drive mode.");
						active_braking = true;
					}

					platform_controller->setActiveBrakeMode(active_braking);	
					break;
				case 'x':
					stop = true;
					break;
		    }

		    platform_controller->writeWheelStates();
		}

		platform_controller->update();

		//! @todo OH: move to platform_controller.cpp ? Make generic logger?
		if(log_PID)
		{
			if(pid_log_drive != NULL && pid_log_steer != NULL)
			{
				fprintf(pid_log_drive, "%.2f", ros::Time::now().toSec() - time_start_sec);
				fprintf(pid_log_steer, "%.2f", ros::Time::now().toSec() - time_start_sec);

				// Get debug states
				platform_controller->getWheelUnitsDebugStates();

				for(auto& wheelunit : platform_controller->getWheelUnits())
				{
					//ROS_DEBUG_NAMED(ROS_NAME, "Vel Error: %d | Drive PID out:  %d | Steer PID out: %d", wheelunit.velocity_error_, wheelunit.drive_PID_out_, wheelunit.steer_PID_out_);
					ROS_INFO_NAMED(ROS_NAME, "wheelunit.measured_velocity_: %.4f", wheelunit.measured_velocity_);
					ROS_INFO_NAMED(ROS_NAME, "wheelunit.drive_P_out_: %d", wheelunit.drive_P_out_);
					//fprintf(pid_log_drive, " %d %d", wheelunit.measured_velocity_, wheelunit.drive_PID_out_);
					if(wheelunit.name_ == "FL" || wheelunit.name_ == "BL")
						fprintf(pid_log_drive, " %d %d %d %d %d %.4f %d %s", -wheelunit.velocity_error_, -wheelunit.drive_PID_out_, -wheelunit.drive_P_out_, -wheelunit.drive_I_out_, -wheelunit.drive_D_out_, wheelunit.measured_velocity_/1000, wheelunit.set_velocity_/1000, wheelunit.name_.c_str());
					else
						fprintf(pid_log_drive, " %d %d %d %d %d %.4f %d %s", wheelunit.velocity_error_, wheelunit.drive_PID_out_, wheelunit.drive_P_out_, wheelunit.drive_I_out_, wheelunit.drive_D_out_, wheelunit.measured_velocity_/1000, wheelunit.set_velocity_/1000, wheelunit.name_.c_str());
					

					fprintf(pid_log_steer, " %d %d %s", wheelunit.position_error_, wheelunit.steer_PID_out_, wheelunit.name_.c_str());
				}
				fprintf(pid_log_drive, "\n");
				fprintf(pid_log_steer, "\n");
			}
			else
				ROS_WARN_NAMED(ROS_NAME, "Could not open PID log file.");
		}

		ros::spinOnce();
		r.sleep();

		ros::Time end = ros::Time::now();
		ros::Duration d = end - begin; 
		// ROS_INFO_NAMED(ROS_NAME, "WC Rate: %.2f", 1.0/d.toSec());	
	}

	if(pid_log_drive != NULL)
		fclose(pid_log_drive);

	if(pid_log_steer != NULL)
		fclose(pid_log_steer);

	delete platform_controller; 

	return 0;
}
