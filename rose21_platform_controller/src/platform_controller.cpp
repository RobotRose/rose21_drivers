/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2013/12/17
* 		- File created.
*
* Description:
*	Controller that communicates using a Serial interface with the propeller that 
* 	contols the wheels. It accepts wheel speeds and orientations and sends it to the
* 	wheels.
* 
***********************************************************************************/

//! @ todo:OH add braking support to platform_controller

#include "rose21_platform_controller/platform_controller.hpp"

using namespace std;

PlatformController::PlatformController(string name, ros::NodeHandle n, string serial_port, int baudrate)
	: HardwareController<Serial>()
    , sh_platform_controller_alarm_(SharedVariable<bool>("platform_controller_alarm"))
    , sh_platform_controller_reset_(SharedVariable<bool>("platform_controller_reset"))
    , sh_emergency_(SharedVariable<bool>("emergency"))
    , velocity_watchdog_("platform_controller_velocity_watchdog", n, VELOCITY_TIMEOUT, boost::bind(&PlatformController::CB_cancelAllMovements, this))
{
    n_           	= n;
    name_       	= name;
    enabled_ 	 	= false;
    comm_interface_	= Serial(name_, serial_port, baudrate);

    // Host read-only, published alarm state, max publish rate is 10 hz
    sh_platform_controller_alarm_.host(false, true, ros::Rate(10.0));

    // Host mutable, non-published reset boolean
    sh_platform_controller_reset_.host(false, false);

    // Connect to state of emercency
    sh_emergency_.connect(ros::Duration(0.1));

    sh_platform_controller_alarm_   = false;
    alarm_number_                   = -1;
    alarm_byte_                     = 0;

    reset_tries_                    = 0;

    smc_ = new SMC(n_, name_, boost::bind(&PlatformController::CB_WheelUnitStatesRequest, this, _1, _2));
    smc_->startServer();

    // Publishers
    wheelunit_states_pub_    = n.advertise<rose_base_msgs::wheelunit_states>("/platform_controller/wheelunit_states", 1);

    // Add Wheelunits to the map
    wheelunits_.push_back( WheelUnit("FR", 0) );
    wheelunits_.push_back( WheelUnit("FL", 2) );
    wheelunits_.push_back( WheelUnit("BR", 4) );
    wheelunits_.push_back( WheelUnit("BL", 6) );

    // Transforms
    wheelunit_transformers_.insert(std::pair<string, TFHelper>("FR",  TFHelper("FR", n_, "/base_link", "FR")));
    wheelunit_transformers_.insert(std::pair<string, TFHelper>("FL",  TFHelper("FL", n_, "/base_link", "FL")));
    wheelunit_transformers_.insert(std::pair<string, TFHelper>("BR",  TFHelper("BR", n_, "/base_link", "BR")));
    wheelunit_transformers_.insert(std::pair<string, TFHelper>("BL",  TFHelper("BL", n_, "/base_link", "BL")));

    ROS_DEBUG_NAMED(ROS_NAME, "Started %s, will use serial connection [%s:%d]", name_.c_str(), serial_port.c_str(), baudrate);  
}

PlatformController::~PlatformController()
{
	disable();
	ROS_INFO_NAMED(ROS_NAME, "Stopped %s", name_.c_str());
}

bool PlatformController::update()
{
    // Do not try to update as long as the emercency button is pressed
    if(sh_emergency_)
    {
        ROS_WARN_THROTTLE_NAMED(5.0, ROS_NAME, "Emergency button pressed, not updating wheelunit states.");        
        return false;
    }

    if(!isEnabled("updating platform controller"))
    {
        // If the platform controller is not enabled try to enable it by resetting
        if(!reset(false))
        {
            alarm_number_ = 6;
            return false;
        }
    }

    // Check the watchdog
    if(!checkWatchdog())
    {
        alarm_number_ = 7;
        ROS_WARN_NAMED(ROS_NAME, "Watchdog error, disabling.");
        disable();
        return false;
    }

    // Update and publish the wheelunit states if succesfull
    if(!getStatus())
        return false;


    publishWheelUnitStates();
    publishWheelUnitTransforms();

    // Debug timers
    // updateTimers();
    // printf("%s\n", getTimersString().c_str());

    if( not alarmStateOk() )
    {
        // If we where persuing a goal wheel state we have to abort at this point.
        smc_->abort();

        return false;
    }
    
    //! @todo OH: Add check to check if wheelstate changed.
    //if(!writeWheelStates())
    //    return false;
            
    return true;
}

bool PlatformController::alarmStateOk()
{
    // Check if we want to reset the alarm state
    if(sh_platform_controller_reset_)
    {
        forceReset();

        // Reset the trigger.
        sh_platform_controller_reset_ = false;
    }

    // Print detailed information about the alarm state
    if(sh_platform_controller_alarm_)
    {

        ROS_ERROR_NAMED(ROS_NAME, "Alarm: %d | Alarm byte(%d): %s", alarm_number_, alarm_byte_, rose_conversions::byteToBinary(alarm_byte_));  //! @todo: OH should this be in the Platform controller class
        

        std::string bin(rose_conversions::byteToBinary(alarm_byte_));
        std::string msg = "Interne communicatie fout (QiK, " + bin + ") platform controller.";

        switch(alarm_number_)
        {
            case 1:         // FE error
                ROS_ERROR_NAMED(ROS_NAME, "FE error received, disabling.");
                disable();
                operator_gui.warn("Volgfout platform controller.");
                return false;
                break;
            case 2:         // Current error
                ROS_ERROR_NAMED(ROS_NAME, "Current error received, disabling.");
                for(auto wheelunit : wheelunits_)
                {
                    getWheelUnitDriveCurrent(wheelunit);
                    getWheelUnitMaxDriveCurrent(wheelunit);
                    getWheelUnitSteerCurrent(wheelunit);
                    getWheelUnitMaxSteerCurrent(wheelunit);
                    ROS_WARN_NAMED(ROS_NAME, "Wheel %s drive current: %4d/%4d of max current: %4d, steer current: %4d/%4d of max current: %4d", wheelunit.name_.c_str(), 
                        wheelunit.measured_drive_current_, 
                        wheelunit.measured_max_drive_current_, 
                        PLATFORM_CONTROLLER_DRIVE_MAX_CURR, 
                        wheelunit.measured_steer_current_, 
                        wheelunit.measured_max_steer_current_,
                        PLATFORM_CONTROLLER_STEER_MAX_CURR);
                }
                operator_gui.warn("Stroom fout platform controller.");
                disable();
                return false;
                break;
            case 3:         // Watchdog error
                ROS_ERROR_NAMED(ROS_NAME, "Watchdog error received, disabling.");
                operator_gui.warn("Communicatie fout (WD) platform controller.");
                disable();
                return false;
                break;
            case 4:         // Connection error
                ROS_ERROR_NAMED(ROS_NAME, "Connection error received, disabling.");
                operator_gui.warn(msg);
                disable();
                return false;
                break;
            case 5:         // MAE error
                ROS_ERROR_NAMED(ROS_NAME, "MAE error received, disabling.");
                operator_gui.warn("Interne communicatie fout (MAE) platform controller.");
                disable();
                return false;
                break;
            default:        // Unkown error
                ROS_ERROR_NAMED(ROS_NAME, "Unkown error number received, disabling.");  
                operator_gui.warn("Onbekende fout platform controller.");
                disable();
                return false;
                break;
        }
    }

    return true;
}


bool PlatformController::enable()
{
	if(enabled_)
		return true;

    sh_platform_controller_alarm_   = false;
    alarm_number_                   = 0;
    alarm_byte_                     = 0;

    // Start watching for responses
    spawnReadloop();

    if(!checkControllerID(PLATFORM_CONTROLLER_FIRMWARE_ID)) 
        return false;

    if(!checkFirmwareVersion(PLATFORM_CONTROLLER_FIRMWARE_MAJOR_VERSION, PLATFORM_CONTROLLER_FIRMWARE_MINOR_VERSION))
        return false;

    ROS_INFO_NAMED(ROS_NAME, "Platform controller with firmware version %d.%d connected.", received_firmware_major_version_, received_firmware_minor_version_);

    if(!resetLowLevel())
    {
        ROS_WARN_NAMED(ROS_NAME, "Unable to reset lowlevel controller.");
        return false;
    }

    // Set wheel speeds to zero and write to controller
    for(auto& wheelunit : wheelunits_)
    {
        wheelunit.setAngleRad(0.0);
        wheelunit.setVelocityRadPerSec(0.0);   
    }

    if(!writeWheelStates())
    {
    	ROS_WARN_NAMED(ROS_NAME, "Unable to set wheel unit states.");
    	return false;
    }

    if(!setErrorTresholds(  PLATFORM_CONTROLLER_FOLLOWING_ERR_TIMER,
                            PLATFORM_CONTROLLER_CURRENT_ERR_TIMER, 
                            PLATFORM_CONTROLLER_CONNECTION_ERR_TIMER,
                            PLATFORM_CONTROLLER_MAE_ERR_TIMER))
    {
        ROS_WARN_NAMED(ROS_NAME, "Unable to set error tresholds.");
        return false;

    }

    if(!setDrivePIDs(   PLATFORM_CONTROLLER_DRIVE_KI, 
                        PLATFORM_CONTROLLER_DRIVE_K, 
                        PLATFORM_CONTROLLER_DRIVE_KP, 
                        PLATFORM_CONTROLLER_DRIVE_KD, 
                        PLATFORM_CONTROLLER_DRIVE_ILIMIT, 
                        PLATFORM_CONTROLLER_DRIVE_SCALE, 
                        PLATFORM_CONTROLLER_DRIVE_SCALE, 
                        WHEELUNIT_MAX_VEL, 
                        PLATFORM_CONTROLLER_DRIVE_FEMAX, 
                        PLATFORM_CONTROLLER_DRIVE_MAX_CURR))
    {
    	ROS_WARN_NAMED(ROS_NAME, "Unable to set drive motor controller PI values.");
    	return false;
    }

    if(!setSteerPIDs(   PLATFORM_CONTROLLER_STEER_KI, 
                        PLATFORM_CONTROLLER_STEER_K, 
                        PLATFORM_CONTROLLER_STEER_KP, 
                        PLATFORM_CONTROLLER_STEER_KD, 
                        PLATFORM_CONTROLLER_STEER_ILIMIT, 
                        PLATFORM_CONTROLLER_STEER_SCALE, 
                        PLATFORM_CONTROLLER_STEER_SCALE, 
                        STEER_VELOCITY_LIMIT*1000,                   //! @todo OH: UGLY magic number
                        PLATFORM_CONTROLLER_STEER_FEMAX, 
                        PLATFORM_CONTROLLER_STEER_MAX_CURR))
    {
    	ROS_WARN_NAMED(ROS_NAME, "Unable to set steer motor controller PI values.");
    	return false;
    }

    if(!setStartStopValues(WHEELUNIT_START_MOVE_ANGLE_ERR_VAL_LOW_LEVEL, WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL_LOW_LEVEL))
    {
        ROS_WARN_NAMED(ROS_NAME, "Could not set start and stop values.");
        return false;
    }   

    // Enable the lowlevel controller by sending a 1
    ControllerResponse 	response = *(new ControllerResponse(PLATFORM_CONTROLLER_ENABLE, HARDWARE_CONTROL_TIMEOUT));
    response.addExpectedDataItem(ControllerData("1", enabled_, "Unable to enable low-level platform controller."));
    ControllerCommand 	command(PLATFORM_CONTROLLER_ENABLE, response);
    command.addDataItem("1");
            
    if(!executeCommand(command))
        return false;  

    if(!setActiveBrakeMode(false))
    {
        ROS_WARN_NAMED(ROS_NAME, "Could not set passive brake drive mode.");
        return false;
    }

    // Enable the watchdog
    spawnWatchdog();

    ROS_INFO_NAMED(ROS_NAME, "Platform controller enabled.");

    return true;
}

bool PlatformController::disable()
{
    if(enabled_)
    {
        stopAllWheels();

        // Disable the watchdog
        stopWatchdog();  

        velocity_watchdog_.stop();	

        // Disable the lowlevel controller by sending a 0
        ControllerResponse 	response = *(new ControllerResponse(PLATFORM_CONTROLLER_ENABLE, HARDWARE_CONTROL_TIMEOUT));
        response.addExpectedDataItem(ControllerData("0", enabled_, "Unable to disable low-level platform controller"));
        ControllerCommand 	command(PLATFORM_CONTROLLER_ENABLE, response);
        command.addDataItem("0");
                
        if(!executeCommand(command))
        {
        	// Stop watching for responses   
        	stopReadloop();

            comm_interface_.disconnect();
 
            // Force locally disabled
            enabled_ = false;

        	return false;        
        }   

    }

    // Stop watching for responses   
    stopReadloop();

    // Disconnect communitcation interface
    comm_interface_.disconnect();

    // Force locally disabled
    enabled_ = false;

    return true;
}

bool PlatformController::forceReset()
{
    reset_tries_ = 0;
    return reset(true);
}

bool PlatformController::reset(bool force_reset)
{
    
    // reset_tries_ DISABLED!
    reset_tries_ = 0;
    // if( not force_reset && reset_tries_ >= MAX_RESET_TRIES )
    // {
    //     disable();
    //     return false;
    // }
    if(sh_emergency_)
    {
        ROS_WARN_THROTTLE_NAMED(0.25, ROS_NAME, "Not resetting platform controller, due to emergency state."); 
        return false;        
    }

    velocity_watchdog_.stop();


    // Reset the platform controller
    ROS_INFO_NAMED(ROS_NAME, "Resetting platform controller."); 
    operator_gui.message("Platform aansturing wordt herstart...");
    reset_tries_++;  
    disable();
    if(enable())
    {
        ROS_INFO_NAMED(ROS_NAME, "Resetting platform controller succesfull (%d/%d).", reset_tries_, MAX_RESET_TRIES);
        operator_gui.message("Herstart platform aansturing succesvol.");
        if(getStatus())
        {
            if( not sh_platform_controller_alarm_ )
            {
                reset_tries_ = 0;
                operator_gui.message("Herstart platform heeft alarm status opgelost.");
                ROS_INFO_NAMED(ROS_NAME, "Alarm state resolved.");
            }
            else
            {
                operator_gui.message("Herstart platform heeft alarm status niet opgelost.");
                ROS_WARN_NAMED(ROS_NAME, "Alarm state not resolved.");
            }
        }
        
        return true;
    }

    if(reset_tries_ < MAX_RESET_TRIES)
    {
        ROS_ERROR_NAMED(ROS_NAME, "Resetting platform controller unsuccesfull (%d/%d), retrying in %.2fs.", reset_tries_, MAX_RESET_TRIES, (float)reset_tries_);
        ros::Duration((float)reset_tries_).sleep();
    }
    else
    {
        operator_gui.warn("Herstart platform aansturing niet gelukt.");
        ROS_ERROR_NAMED(ROS_NAME, "Resetting platform controller unsuccesfull (%d/%d), disabling platform controller.", reset_tries_, MAX_RESET_TRIES);
    }   

    return false;
}

bool PlatformController::isEnabled(string performing_action)
{
    if(!enabled_ && reset_tries_ >= MAX_RESET_TRIES)
        ROS_WARN_THROTTLE_NAMED(10.0, ROS_NAME, "Platform controller is disabled while %s, consider force resetting.", performing_action.c_str());

	return enabled_;
}

bool PlatformController::stopAllWheels()
{
	for(auto& wheelunit : wheelunits_)
		wheelunit.setVelocityRadPerSec(0.0);

	return writeWheelStates();
}

bool PlatformController::resetLowLevel()
{
    return simpleCommand(PLATFORM_CONTROLLER_RESET_PLATFORM, HARDWARE_CONTROL_TIMEOUT);
}

bool PlatformController::resetLowLevelSafety()
{
    sh_platform_controller_alarm_   = false;
    alarm_number_                   = 0;
    alarm_byte_                     = 0;
    return simpleCommand(PLATFORM_CONTROLLER_RESET_ALARM, HARDWARE_CONTROL_TIMEOUT);
}

bool PlatformController::resetLowLevelCommunication()
{
    return simpleCommand(PLATFORM_CONTROLLER_RESET_COMM, HARDWARE_CONTROL_TIMEOUT);
}

bool PlatformController::setDrivePIDs(	int Ki, 
									int K, 
                                    int Kp, 
									int Kd, 
									int Ilimit, 
									int PosScale, 
									int VelScale, 
									int VelMax,
									int FeMax, 
									int MaxCurr)
{
	if(enabled_)
	{
		ROS_WARN_NAMED(ROS_NAME, "Cannot set drive PID values when controller is enabled.");
		return false;
	}

    ControllerResponse response(PLATFORM_CONTROLLER_DRIVE_PID_VALS, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(Ki,         "Drive PID Ki value incorrectly set."));
    response.addExpectedDataItem(ControllerData(K,          "Drive PID K value incorrectly set."));
    response.addExpectedDataItem(ControllerData(Kp,         "Drive PID Kp value incorrectly set."));
    response.addExpectedDataItem(ControllerData(Kd,         "Drive PID Kd value incorrectly set."));
    response.addExpectedDataItem(ControllerData(Ilimit,     "Drive PID Ilimit value incorrectly set."));
    response.addExpectedDataItem(ControllerData(PosScale,   "Drive PID PosScale value incorrectly set."));
    response.addExpectedDataItem(ControllerData(VelScale,   "Drive PID VelScale value incorrectly set."));
    response.addExpectedDataItem(ControllerData(VelMax,     "Drive PID VelMax value incorrectly set.")); 
    response.addExpectedDataItem(ControllerData(FeMax,      "Drive PID FeMax value incorrectly set."));
    response.addExpectedDataItem(ControllerData(MaxCurr,    "Drive PID MaxCurr value incorrectly set."));
    
    ControllerCommand  command(PLATFORM_CONTROLLER_DRIVE_PID_VALS, response);
    command.addDataItem(Ki);
    command.addDataItem(K);
    command.addDataItem(Kp);
    command.addDataItem(Kd);
    command.addDataItem(Ilimit);
    command.addDataItem(PosScale);
    command.addDataItem(VelScale);
    command.addDataItem(VelMax);
    command.addDataItem(FeMax);
    command.addDataItem(MaxCurr);
    
    return executeCommand(command);
}

bool PlatformController::setSteerPIDs(	int Ki, 
									int K, 
                                    int Kp, 
									int Kd, 
									int Ilimit, 
									int PosScale, 
									int VelScale, 
									int VelMax,
									int FeMax, 
									int MaxCurr)
{
	if(enabled_)
	{
		ROS_WARN_NAMED(ROS_NAME, "Cannot set steer PID values when controller is enabled.");
		return false;
	}

    ControllerResponse response(PLATFORM_CONTROLLER_STEER_PID_VALS, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(Ki,         "Steer PID Ki value incorrectly set."));
    response.addExpectedDataItem(ControllerData(K,          "Steer PID K value incorrectly set."));
    response.addExpectedDataItem(ControllerData(Kp,         "Steer PID Kp value incorrectly set."));
    response.addExpectedDataItem(ControllerData(Kd,         "Steer PID Kd value incorrectly set."));
    response.addExpectedDataItem(ControllerData(Ilimit,     "Steer PID Ilimit value incorrectly set."));
    response.addExpectedDataItem(ControllerData(PosScale,   "Steer PID PosScale value incorrectly set."));
    response.addExpectedDataItem(ControllerData(VelScale,   "Steer PID VelScale value incorrectly set."));
    response.addExpectedDataItem(ControllerData(VelMax,     "Steer PID VelMax value incorrectly set.")); 
    response.addExpectedDataItem(ControllerData(FeMax,      "Steer PID FeMax value incorrectly set."));
    response.addExpectedDataItem(ControllerData(MaxCurr,    "Steer PID MaxCurr value incorrectly set."));
    
    ControllerCommand command(PLATFORM_CONTROLLER_STEER_PID_VALS, response);
    command.addDataItem(Ki);
    command.addDataItem(K);
    command.addDataItem(Kp);
    command.addDataItem(Kd);
    command.addDataItem(Ilimit);
    command.addDataItem(PosScale);
    command.addDataItem(VelScale);
    command.addDataItem(VelMax);   
    command.addDataItem(FeMax);
    command.addDataItem(MaxCurr);
            
    return executeCommand(command);
}

bool PlatformController::setErrorTresholds(const unsigned int& following_err_timer,              
                                        const unsigned int& current_err_timer,
                                        const unsigned int& connection_err_timer,
                                        const unsigned int& mae_err_timer)
{
    ControllerResponse response(PLATFORM_CONTROLLER_ERROR_TIMERS, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(following_err_timer,   "Error timer 'following error' incorrectly set."));
    response.addExpectedDataItem(ControllerData(current_err_timer,     "Error timer 'current error' incorrectly set."));
    response.addExpectedDataItem(ControllerData(connection_err_timer,  "Error timer 'wheel unit connection' incorrectly set."));
    response.addExpectedDataItem(ControllerData(mae_err_timer,         "Error timer 'MAE error' incorrectly set."));

    ControllerCommand  command(PLATFORM_CONTROLLER_ERROR_TIMERS, response);
    command.addDataItem(following_err_timer);
    command.addDataItem(current_err_timer);
    command.addDataItem(connection_err_timer);
    command.addDataItem(mae_err_timer);

    return executeCommand(command);
}

// The start stop values are the values of error when the forward movement is started/stopped when rotating while driving
bool PlatformController::setStartStopValues(int start_value_low_level, int stop_value_low_level)
{
    ROS_DEBUG_NAMED(ROS_NAME, "Setting start/stop rotation error limits to: %d/%d.", start_value_low_level, stop_value_low_level);
    ControllerResponse response(PLATFORM_CONTROLLER_START_STOP_VALS, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(start_value_low_level, "Start movement treshold value incorectly set."));
    response.addExpectedDataItem(ControllerData(stop_value_low_level,  "Stop movement treshold value incorectly set."));
    
    ControllerCommand  command(PLATFORM_CONTROLLER_START_STOP_VALS, response);
    command.addDataItem(start_value_low_level);
    command.addDataItem(stop_value_low_level);

    return executeCommand(command); 
    return true;
}

bool PlatformController::setActiveBrakeMode(bool active_brake)
{

    int active_brake_lowlevel_param = 1;
    if(active_brake)
        active_brake_lowlevel_param = 1;     // Active brake drive mode
    else
        active_brake_lowlevel_param = 0;     // Passive brake drive mode

    ControllerResponse response(PLATFORM_CONTROLLER_BRAKE_MODE, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(active_brake_lowlevel_param, "Drive/brake mode incorrectly set, is the controller enabled?"));
    
    ControllerCommand  command(PLATFORM_CONTROLLER_BRAKE_MODE, response);
    command.addDataItem(active_brake_lowlevel_param);

    return executeCommand(command); 
}


bool PlatformController::getWheelUnitDrivePosition(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit drive position"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;
    
    // Drive motor
    // Get new position
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_POSITION, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.measured_drive_encoder_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_POSITION, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;

    // Store the time of measurement
    wheelunit.measured_drive_encoder_time_ = ros::Time::now();

    return true;
}

bool PlatformController::getAllWheelUnitDriveEncoderSpeeds()
{    
    if(!isEnabled("getting all wheelunits encoder speeds"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;
    
    // Drive get encoder differences and calculate motor
    wheelunits_[0].measured_drive_encoder_diff_ = 0;
    wheelunits_[1].measured_drive_encoder_diff_ = 0; 
    wheelunits_[2].measured_drive_encoder_diff_ = 0;
    wheelunits_[3].measured_drive_encoder_diff_ = 0;
    int clock_diff_CNT = 0;
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_ALL_DRIVE_ENC_DIFF, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunits_[0].measured_drive_encoder_diff_));
    response->addExpectedDataItem(ControllerData(wheelunits_[1].measured_drive_encoder_diff_));
    response->addExpectedDataItem(ControllerData(wheelunits_[2].measured_drive_encoder_diff_));
    response->addExpectedDataItem(ControllerData(wheelunits_[3].measured_drive_encoder_diff_));
    response->addExpectedDataItem(ControllerData(clock_diff_CNT));    
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_ALL_DRIVE_ENC_DIFF, *response);

    if(!executeCommand(*command))
        return false;
    
    float dT = (float)clock_diff_CNT/(float)PLATFORM_CONTROLLER_CLK_FREQ;

    if(clock_diff_CNT == 0)
    {
        ROS_WARN_NAMED(ROS_NAME, "Encoder difference was zero.");
        dT = 0.0;
    }
    else
    {       
        wheelunits_[0].measured_velocity_ = ((float)wheelunits_[0].measured_drive_encoder_diff_)/dT;
        wheelunits_[1].measured_velocity_ = ((float)wheelunits_[1].measured_drive_encoder_diff_)/dT; 
        wheelunits_[2].measured_velocity_ = ((float)wheelunits_[2].measured_drive_encoder_diff_)/dT;
        wheelunits_[3].measured_velocity_ = ((float)wheelunits_[3].measured_drive_encoder_diff_)/dT;
    }

    wheelunits_[0].dT_                = dT;
    wheelunits_[1].dT_                = dT;
    wheelunits_[2].dT_                = dT;
    wheelunits_[3].dT_                = dT;

    return true;
}

bool PlatformController::getWheelUnitSteerVelocity(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit steer velocity"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;
    
    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_VELOCITY, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.encoder_measured_velocity_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_VELOCITY, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    ROS_WARN_NAMED(ROS_NAME, "HERE");

    return true;
}

bool PlatformController::getWheelUnitSteerPosition(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit steer position"))
        return false;

	ControllerResponse* 	response;
	ControllerCommand* 		command;
	
	// Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_POSITION, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.measured_rotation_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_POSITION, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;
}

bool PlatformController::getWheelUnitVelocityError(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit velocity error"))
        return false;

	ControllerResponse* 	response;
	ControllerCommand* 		command;

	// Drive motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_VEL_ERROR, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.velocity_error_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_VEL_ERROR, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;  
}

bool PlatformController::getWheelUnitPositionError(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit postion error"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_POS_ERROR, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.position_error_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_POS_ERROR, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;
}

bool PlatformController::getWheelUnitPIDOut(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit PID output signal"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Drive motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_PIDOUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.drive_PID_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_PIDOUT, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;  

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_PIDOUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.steer_PID_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_PIDOUT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitPOut(WheelUnit& wheelunit)
{
    if(!isEnabled("getting P output signal"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Drive motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_POUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.drive_P_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_POUT, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;  

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_POUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.steer_P_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_POUT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitIOut(WheelUnit& wheelunit)
{
    if(!isEnabled("getting I output signal"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Drive motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_IOUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.drive_I_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_IOUT, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;  

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_IOUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.steer_I_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_IOUT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitDOut(WheelUnit& wheelunit)
{
    if(!isEnabled("getting D output signal"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Drive motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_DOUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.drive_D_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_DOUT, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;  

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_DOUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.steer_D_out_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_DOUT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitMAE(WheelUnit& wheelunit)
{
    if(!isEnabled("getting MAE value"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_MAE_OUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.MAE_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_MAE_OUT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitFE(WheelUnit& wheelunit)
{
    if(!isEnabled("getting following error"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_FE_OUT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.FE_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_FE_OUT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitDriveCurrent(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit drive current"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_CURRENT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.measured_drive_current_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_CURRENT, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitSteerCurrent(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit steer current"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_CURRENT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.measured_steer_current_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_CURRENT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitMaxDriveCurrent(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit maximal drive current"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_MAX_CURRENT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.measured_max_drive_current_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_MAX_CURRENT, *response);
    command->addDataItem(wheelunit.getDriveMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitMaxSteerCurrent(WheelUnit& wheelunit)
{
    if(!isEnabled("getting wheelunit maximal steer current"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    // Steer motor
    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_MAX_CURRENT, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(wheelunit.measured_max_steer_current_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_MAX_CURRENT, *response);
    command->addDataItem(wheelunit.getSteerMotorID());

    if(!executeCommand(*command))
        return false;  

    return true;    
}

bool PlatformController::getWheelUnitsDebugStates()
{
    if(!isEnabled("getting complete debug status"))
        return false;

    ControllerResponse response(PLATFORM_CONTROLLER_GET_DEBUG_STATUS, HARDWARE_CONTROL_TIMEOUT);

    response.addExpectedDataItem(ControllerData(wheelunits_[0].velocity_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].position_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].drive_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].steer_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].drive_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].steer_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].drive_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].steer_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].drive_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].steer_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].FE_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].measured_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].measured_max_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].measured_steer_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[0].measured_max_steer_current_));

    response.addExpectedDataItem(ControllerData(wheelunits_[1].velocity_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].position_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].drive_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].steer_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].drive_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].steer_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].drive_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].steer_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].drive_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].steer_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].FE_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].measured_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].measured_max_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].measured_steer_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[1].measured_max_steer_current_));

    response.addExpectedDataItem(ControllerData(wheelunits_[2].velocity_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].position_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].drive_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].steer_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].drive_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].steer_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].drive_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].steer_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].drive_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].steer_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].FE_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].measured_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].measured_max_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].measured_steer_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[2].measured_max_steer_current_));

    response.addExpectedDataItem(ControllerData(wheelunits_[3].velocity_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].position_error_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].drive_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].steer_PID_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].drive_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].steer_P_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].drive_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].steer_I_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].drive_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].steer_D_out_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].FE_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].measured_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].measured_max_drive_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].measured_steer_current_));
    response.addExpectedDataItem(ControllerData(wheelunits_[3].measured_max_steer_current_));

    ControllerCommand command(PLATFORM_CONTROLLER_GET_DEBUG_STATUS, response);

    return executeCommand(command);
}

// alarm = 1 if there is an alarm, 0 if no alarm. 
// alarm_number then indicates which alarm
bool PlatformController::getAlarmStatus()
{
    if(!isEnabled("getting alarm state"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;
    int alarm_int = 0;    

    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_ALARM_STATE, HARDWARE_CONTROL_TIMEOUT);
    response->addExpectedDataItem(ControllerData(alarm_int));
    response->addExpectedDataItem(ControllerData(alarm_number_));
    response->addExpectedDataItem(ControllerData(alarm_byte_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_ALARM_STATE, *response);

    if(!executeCommand(*command))
        return false;  

    ROS_INFO_NAMED(ROS_NAME, "Alarm: %d", alarm_int);

    if(alarm_int == 1)
        sh_platform_controller_alarm_ = true;
    else
        sh_platform_controller_alarm_ = false;
    
    return true;    
}    

bool PlatformController::getStatus()
{
    if(!isEnabled("getting status report"))
        return false;

    ControllerResponse*     response;
    ControllerCommand*      command;

    response = new ControllerResponse(PLATFORM_CONTROLLER_GET_COMPLETE_STATUS, HARDWARE_CONTROL_TIMEOUT);
    
    int clock_diff_CNT = 0;
    // Difference counts
    for(auto& wheelunit : wheelunits_)
    {
        wheelunit.measured_drive_encoder_diff_ = 0;
        response->addExpectedDataItem(ControllerData(wheelunit.measured_drive_encoder_diff_));
    }
    // Delta clock cycles
    response->addExpectedDataItem(ControllerData(clock_diff_CNT));    
    
    // Steer encoder values
    for(auto& wheelunit : wheelunits_)
        response->addExpectedDataItem(ControllerData(wheelunit.measured_rotation_));

    // Alarm state
    int alarm_int = 0;
    response->addExpectedDataItem(ControllerData(alarm_int));
    response->addExpectedDataItem(ControllerData(alarm_number_));
    response->addExpectedDataItem(ControllerData(alarm_byte_));
    command  = new ControllerCommand(PLATFORM_CONTROLLER_GET_COMPLETE_STATUS, *response);

    // Send command and wait for response
    if(!executeCommand(*command))
        return false;  


    // Drive wheel encoder values post processing
    float dT = (float)clock_diff_CNT/(float)PLATFORM_CONTROLLER_CLK_FREQ;              //! @todo OH: Duplicate code with getAllWheelUnitDriveEncoderSpeeds

    if(clock_diff_CNT == 0)
    {
        ROS_WARN_NAMED(ROS_NAME, "Encoder difference (dT) is zero."); 
        dT = 0.0;
    }
    else if(clock_diff_CNT < 0)
    {
        ROS_WARN_NAMED(ROS_NAME, "Encoder difference (dT) is < zero, this should not happen?"); 
        dT = 0.0;
    }
    else
    {       
        // Calculate velocity only when dt is not zero
        for(auto& wheelunit : wheelunits_)
            wheelunit.measured_velocity_ = ((float)wheelunit.measured_drive_encoder_diff_)/dT;
    }

    // Store dt
    for(auto& wheelunit : wheelunits_)
        wheelunit.dT_ = dT;
        
    // Alarm state post processing
    if(alarm_int == 1)
        sh_platform_controller_alarm_ = true;
    else if(alarm_int == 0)
        sh_platform_controller_alarm_ = false; 

    return true;    
}

vector<WheelUnit>& PlatformController::getWheelUnits()
{
	return wheelunits_;
}

bool PlatformController::sendWheelState()
{
     // If we have an alarm state, return false
    if(sh_platform_controller_alarm_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Platform has raised the alarm state, will not write the requested wheelunit state.");
        return false;
    }
    
    // Write to the low-level controller
    return writeWheelStates();
}

bool PlatformController::writeWheelStates()
{
    WheelUnit wheelunit;
    ControllerResponse  response = *new ControllerResponse(PLATFORM_CONTROLLER_MOVE, HARDWARE_CONTROL_TIMEOUT);
    ControllerCommand   command(PLATFORM_CONTROLLER_MOVE);

    for(auto& wheelunit : wheelunits_)
    {
        response.addExpectedDataItem(ControllerData(wheelunit.getSetVelocityLowLevel(), wheelunit.set_velocity_));
        response.addExpectedDataItem(ControllerData(wheelunit.getSetAngleLowLevel(), wheelunit.set_rotation_));
        command.addDataItem(wheelunit.getSetVelocityLowLevel());
        command.addDataItem(wheelunit.getSetAngleLowLevel());
    }
    command.setExpectedResponse(response);

    if(!executeCommand(command))
    {
        wheelunits_ = prev_wheelunits_;
        return false;    
    }

    // Stop the watchdog if we succesfully have written a stop command
    if(stopWritten())
        velocity_watchdog_.stop();

    // Store this succesfully written state
    prev_wheelunits_ = wheelunits_;

    return true;
}

void PlatformController::publishWheelUnitStates()
{

    //! @todo OH: HACK HARD CODED INDEXES!

    // Fill message with lowlevel values
    rose_base_msgs::wheelunit_states wheelunit_states; 
    wheelunit_states.angle_FR       = wheelunits_[0].measured_rotation_;   
    wheelunit_states.angle_FL       = wheelunits_[1].measured_rotation_;
    wheelunit_states.angle_BR       = wheelunits_[2].measured_rotation_;
    wheelunit_states.angle_BL       = wheelunits_[3].measured_rotation_;

    wheelunit_states.velocity_FR    = wheelunits_[0].measured_velocity_;
    wheelunit_states.velocity_FL    = wheelunits_[1].measured_velocity_;
    wheelunit_states.velocity_BR    = wheelunits_[2].measured_velocity_;
    wheelunit_states.velocity_BL    = wheelunits_[3].measured_velocity_;

    wheelunit_states.diff_FR        = wheelunits_[0].measured_drive_encoder_diff_;
    wheelunit_states.diff_FL        = wheelunits_[1].measured_drive_encoder_diff_;
    wheelunit_states.diff_BR        = wheelunits_[2].measured_drive_encoder_diff_;
    wheelunit_states.diff_BL        = wheelunits_[3].measured_drive_encoder_diff_;

    wheelunit_states.dT_FR          = wheelunits_[0].dT_;
    wheelunit_states.dT_FL          = wheelunits_[1].dT_;
    wheelunit_states.dT_BR          = wheelunits_[2].dT_;
    wheelunit_states.dT_BL          = wheelunits_[3].dT_;

    // Create the wheelunits and add them to a map for easier access later on
    map<string, WheelUnit> wheelunits_map_;
    WheelUnit wheel_unit_FL("FR", 0);
    WheelUnit wheel_unit_FR("FL", 2);
    WheelUnit wheel_unit_BL("BR", 4);
    WheelUnit wheel_unit_BR("BL", 6);
    wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_FL.name_, wheel_unit_FL) );
    wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_FR.name_, wheel_unit_FR) );
    wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_BL.name_, wheel_unit_BL) );
    wheelunits_map_.insert( std::pair<string, WheelUnit>(wheel_unit_BR.name_, wheel_unit_BR) );



    wheelunits_map_.at("FR").measured_rotation_             = wheelunits_[0].measured_rotation_;
    wheelunits_map_.at("FL").measured_rotation_             = wheelunits_[1].measured_rotation_;
    wheelunits_map_.at("BR").measured_rotation_             = wheelunits_[2].measured_rotation_;
    wheelunits_map_.at("BL").measured_rotation_             = wheelunits_[3].measured_rotation_;
    wheelunits_map_.at("FR").measured_velocity_             = wheelunits_[0].measured_velocity_;
    wheelunits_map_.at("FL").measured_velocity_             = wheelunits_[1].measured_velocity_;
    wheelunits_map_.at("BR").measured_velocity_             = wheelunits_[2].measured_velocity_;
    wheelunits_map_.at("BL").measured_velocity_             = wheelunits_[3].measured_velocity_;
    wheelunits_map_.at("FR").measured_drive_encoder_diff_   = wheelunits_[0].measured_drive_encoder_diff_; 
    wheelunits_map_.at("FL").measured_drive_encoder_diff_   = wheelunits_[1].measured_drive_encoder_diff_;
    wheelunits_map_.at("BR").measured_drive_encoder_diff_   = wheelunits_[2].measured_drive_encoder_diff_;
    wheelunits_map_.at("BL").measured_drive_encoder_diff_   = wheelunits_[3].measured_drive_encoder_diff_; 
    wheelunits_map_.at("FR").dT_                            =  wheelunits_[0].dT_;
    wheelunits_map_.at("FL").dT_                            =  wheelunits_[1].dT_;
    wheelunits_map_.at("BR").dT_                            =  wheelunits_[2].dT_;
    wheelunits_map_.at("BL").dT_                            =  wheelunits_[3].dT_;

    wheelunit_states_pub_.publish(wheelunit_states);
}


void PlatformController::publishWheelUnitTransforms()
{
    // Set broadcasts
    wheelunit_transformers_.at("FR").setTransform(0.0, 0.0, -(wheelunits_[0].getMeasuredAngleRad()), 0.58/2.0, -0.48/2.0, 0.0);
    wheelunit_transformers_.at("FL").setTransform(0.0, 0.0, -(wheelunits_[1].getMeasuredAngleRad()), 0.58/2.0, 0.48/2.0, 0.0); 
    wheelunit_transformers_.at("BR").setTransform(0.0, 0.0, -(wheelunits_[2].getMeasuredAngleRad()), -0.58/2.0, -0.48/2.0, 0.0);
    wheelunit_transformers_.at("BL").setTransform(0.0, 0.0, -(wheelunits_[3].getMeasuredAngleRad()), -0.58/2.0, 0.48/2.0, 0.0);
    
    // Broadcast!
    for(auto&  wheelunit_transformer : wheelunit_transformers_)
    {
        wheelunit_transformer.second.Broadcast();
    }
}

void PlatformController::CB_WheelUnitStatesRequest(const rose_base_msgs::wheelunit_statesGoalConstPtr& goal, SMC* smc)
{
    // Do not try to update as long as the emercency button is pressed
    if(sh_emergency_)
    {
        ROS_WARN_THROTTLE_NAMED(5.0, ROS_NAME, "Emergency button pressed, not updating wheelunit states.");        
        return;
    }

    // ROS_INFO_NAMED(ROS_NAME, "Set wheel unit state request received.");

    velocity_watchdog_.reset();

    // Transfer data to the wheel units 
    wheelunits_[0].set_rotation_ = goal->requested_state.angle_FR;
    wheelunits_[1].set_rotation_ = goal->requested_state.angle_FL;
    wheelunits_[2].set_rotation_ = goal->requested_state.angle_BR;
    wheelunits_[3].set_rotation_ = goal->requested_state.angle_BL;

    wheelunits_[0].set_velocity_ = goal->requested_state.velocity_FR;
    wheelunits_[1].set_velocity_ = goal->requested_state.velocity_FL;
    wheelunits_[2].set_velocity_ = goal->requested_state.velocity_BR;
    wheelunits_[3].set_velocity_ = goal->requested_state.velocity_BL;


    rose_base_msgs::wheelunit_statesResult server_result;

    if(sendWheelState())  //!  @todo OH what if canceled in the mean time?
    {
        ROS_DEBUG_NAMED(ROS_NAME, "Successfully set requested wheelunits states.");

        server_result.return_code = SUCCESS;
        server_result.error_code  = 0;
        smc_->sendServerResult(true, server_result);
    }
    else
    {
        ROS_WARN_NAMED(ROS_NAME, "Failed to set requested wheelunits states, error_code: %d", alarm_number_);

        server_result.return_code = LOWLEVEL_ERROR;
        server_result.error_code  = alarm_number_;
        smc_->sendServerResult(false, server_result);
    }
}

void PlatformController::CB_cancelAllMovements()
{
    velocity_watchdog_.stop();

    // Put all states to 0
    wheelunits_[0].set_rotation_ = 0;
    wheelunits_[1].set_rotation_ = 0;
    wheelunits_[2].set_rotation_ = 0;
    wheelunits_[3].set_rotation_ = 0;
    wheelunits_[0].set_velocity_ = 0;
    wheelunits_[1].set_velocity_ = 0;
    wheelunits_[2].set_velocity_ = 0;
    wheelunits_[3].set_velocity_ = 0;

    // If we are not able to write the wheelstate, raise the emergency state
    int retries = 2;
    for(int i = 0; i < retries; i++)
    {
        // If succesfully written the written the wheelstate, return
        if(sendWheelState())
            return;
    }

    // If not succesfull, raise the emergency state
    ROS_WARN_NAMED(ROS_NAME, "Could not stop wheels on watchdog, raising the emergency state.");
    sh_emergency_ = true;
}

bool PlatformController::stopWritten()
{
    return setAnglesZero() and setVelocitiesZero();
}

bool PlatformController::setAnglesZero()
{
    for(int i = 0; i < 4; i++)
    {
        if( wheelunits_[i].set_rotation_ != 0 )
            return false;
    }

    return true; 
}

bool PlatformController::setVelocitiesZero()
{
    for(int i = 0; i < 4; i++)
    {
        if( wheelunits_[i].set_velocity_ != 0 )
            return false;
    }

    return true; 
}
