/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2013/12/10
* 		- File created.
*
* Description:
*	The lift contoller class
* 
***********************************************************************************/

#include "lift_controller/lift_controller.hpp"

using namespace std;

LiftController::LiftController(string name, ros::NodeHandle n, string serial_port, int baudrate)
    : HardwareController<Serial>()
    , controller_enabled_(false)
    , requested_controller_enabled_(false)
    , sh_emergency_(SharedVariable<bool>("emergency"))
{
    n_                      = n;
    name_                   = name;
    comm_interface_         = Serial(name, serial_port, baudrate);

    // Publishers
    lift_pub_               = n_.advertise<rose21_platform::lift_state>("/lift_controller/lift/state", 1, true);
    bumpers_pub_            = n_.advertise<rose21_platform::bumpers_state>("/lift_controller/bumpers/state", 1, true);
    bumpers2_pub_           = n_.advertise<contact_sensor_msgs::bumpers>("/lift_controller/bumpers2/state", 1, true);

    // Subscribers
    lift_controller_enable_sub_  = n_.subscribe("/lift_controller/enabled", 1, &LiftController::CB_SetControllerState, this);
    lift_position_request_sub_   = n_.subscribe("/lift_controller/lift/position_request", 1, &LiftController::CB_LiftPositionRequest, this);

    sh_emergency_.host(false);
    sh_emergency_ = false;

    ROS_INFO_NAMED(ROS_NAME, "Started %s", name_.c_str());

    // Get the bumper footprints
    //n_p_.getParam("bumper_footprints", bumper_footprints_);
    //! @todo OH: HACK MAKE BUMPER CLASS? OR NOT? DISCUSS, MAKE CONFIGURABLE VIA YAML
    std::vector<rose_geometry::Point> bumper;
    rose_geometry::Point c;
    float x_size;
    float y_size;
    float bumper_width = 0.12/2.0;
    float bumper_thickness = 0.02/2.0 + 0.05; 


    x_size = bumper_thickness;
    y_size = bumper_width;
    c = rose_geometry::Point(0.4166, 0.190, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[0] = bumper;
    bumper.clear();
    
    x_size = bumper_width;
    y_size = bumper_thickness;
    c = rose_geometry::Point(0.284, 0.315, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[1] = bumper;
    bumper.clear();
    
    x_size = bumper_width;
    y_size = bumper_thickness;
    c = rose_geometry::Point(-0.284, 0.315, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[2] = bumper;
    bumper.clear();
    
    x_size = bumper_thickness;
    y_size = 0.5/2.0;
    c = rose_geometry::Point(-0.4036, 0.0, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[3] = bumper;
    bumper.clear();
    
    
    x_size = bumper_width;
    y_size = bumper_thickness;
    c = rose_geometry::Point(-0.284, -0.315, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[4] = bumper;
    bumper.clear();
    
    x_size = bumper_width;
    y_size = bumper_thickness;
    c = rose_geometry::Point(0.284, -0.315, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[5] = bumper;
    bumper.clear();
    
    x_size = bumper_thickness;
    y_size = bumper_width;
    c = rose_geometry::Point(0.4166, -0.190, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[6] = bumper;
    bumper.clear();

    x_size = bumper_thickness;
    y_size = bumper_width;
    c = rose_geometry::Point(0.4681, 0.0, 0.0);
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y + y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x - x_size, c.y - y_size, 0.0));
    bumper.push_back(rose_geometry::Point(c.x + x_size, c.y - y_size, 0.0));
    bumper_footprints_[7] = bumper;
    bumper.clear();


    enable();
}

LiftController::~LiftController()
{
    disable();

    ROS_INFO_NAMED(ROS_NAME, "Stopped %s", name_.c_str());
}

void LiftController::publishLiftState()
{
    rose21_platform::lift_state lift_state;
    lift_state.target_position_percentage   = set_lift_position_percentage_;
    lift_state.position_percentage          = cur_position_percentage_;
    lift_state.moving                       = is_moving_;
    lift_state.in_position                  = is_in_position_;
     
    lift_pub_.publish(lift_state);

    ROS_DEBUG_NAMED(ROS_NAME, "Published lift status.");
}

void LiftController::publishBumpersState()
{
    rose21_platform::bumpers_state bumpers_state;
    bumpers_state.bumper_count    = 8;
    for(int i = 0; i < 8; i++)
        if(bumper_states_[i] == 1)
            bumpers_state.bumper_states.push_back(true);
        else
            bumpers_state.bumper_states.push_back(false);

    bumpers_pub_.publish(bumpers_state);


    //! @todo OH: HACK
    contact_sensor_msgs::bumper                 bumper;
    contact_sensor_msgs::bumpers                bumpers_msg;
    geometry_msgs::PolygonStamped               stamped_footprint;

    int i = 0;
    for(const auto& bumper_footprint : bumper_footprints_)
    {
        stamped_footprint.header.stamp      = ros::Time::now();
        stamped_footprint.header.frame_id   = "base_link";
        stamped_footprint.polygon.points    = toROSmsgs32(bumper_footprint.second);

        bumper.footprint    = stamped_footprint;
        bumper.state        = ros::conversion::convert<bool>().get(bumper_states_[i++] != 0);
        bumpers_msg.bumpers.push_back(bumper);
    }   

    bumpers2_pub_.publish(bumpers_msg);

    ROS_DEBUG_NAMED(ROS_NAME, "Published bumpers status.");

    // Reset/start communication
    // resetComm();  
}

bool LiftController::enable()
{      
    // Check if enabled
    if(controller_enabled_)
        return true;   

    ROS_INFO_NAMED(ROS_NAME, "Enabling lift and bumper controller.");
    if(!get_comm_interface()->connect())   
        return false;

    // Reset/start communication
    resetComm();  

    if(!checkControllerID(LIFT_CONTROL_FIRMWARE_ID)) 
        return false;

    if(!checkFirmwareVersion(LIFT_CONTROL_FIRMWARE_MAJOR_VERSION, LIFT_CONTROL_FIRMWARE_MINOR_VERSION)) 
        return false;

    ROS_INFO_NAMED(ROS_NAME, "Lift and bumper controller controller with firmware version %d.%d connected.", received_firmware_major_version_, received_firmware_minor_version_);


    if(!setDefaults())
    {
        ROS_WARN_NAMED(ROS_NAME, "Could not set default parameters for lift and bumper controller, controller has not enabled.");
    }

    int controller_enabled_integer = 0;
    setValue(LIFT_CONTROLLER_ENABLE, HARDWARE_CONTROL_TIMEOUT, 1, controller_enabled_integer);

    if (controller_enabled_integer == 1)
    {
        controller_enabled_ = true;      
        ROS_INFO_NAMED(ROS_NAME, "Lift and bumper controller enabled."); 
    }
    else
    {
        controller_enabled_ = false;
        ROS_WARN_NAMED(ROS_NAME, "Could not enable lift and bumper controller."); 
    }

    return controller_enabled_;
}

bool LiftController::disable()
{
    // Check if enabled
    if(!controller_enabled_)
        return true;   

    // Disable the watchdog
    stopWatchdog();         

    int controller_enabled_integer = 0;
    setValue(LIFT_CONTROLLER_ENABLE, HARDWARE_CONTROL_TIMEOUT, 0, controller_enabled_integer);

    // Stop reading the serial interface
    stopReadloop();  

    if (controller_enabled_integer  == 0)
    {
        controller_enabled_ = false;
        ROS_INFO_NAMED(ROS_NAME, "Lift and bumper controller disabled."); 
    }
    else
    {
        controller_enabled_ = true;
        ROS_WARN_NAMED(ROS_NAME, "Could not disable lift and bumper controller."); 
    }

    return controller_enabled_;
}


bool LiftController::isEnabled()
{
    return controller_enabled_;
}

void LiftController::resetState()
{
    bumper_states_[0]               = 0;
    bumper_states_[1]               = 0;
    bumper_states_[2]               = 0;
    bumper_states_[3]               = 0;
    bumper_states_[4]               = 0;
    bumper_states_[5]               = 0;
    bumper_states_[6]               = 0;
    bumper_states_[7]               = 0;
    safety_state_[0]                = 0;
    safety_state_[1]                = 0;
    safety_state_[2]                = 0;
    safety_state_[3]                = 0;
    safety_state_[4]                = 0;
    safety_state_[5]                = 0;
    controller_enabled_             = 0;  
    safety_input_state_             = 0;
    extra_input_state_              = 0;
    extra_output_state_             = 0;
    // float   adc_raw_[16][4];
    // float   adc_vol_[16][4];
    // float   adc_eng_[16][4];
    // float   adc_int_[16][2];

    min_lift_position_              = 200;
    max_lift_position_              = 3400;
    min_lift_speed_                 = 0;
    max_lift_speed_                 = 255;
    set_lift_speed_                 = 0;
    set_lift_speed_percentage_      = 0;
    set_lift_position_              = 0;
    set_lift_position_percentage_   = 0;
    cur_position_                   = -1;
    cur_position_error_             = -1;
    cur_position_percentage_        = -1; 
    is_in_position_                 = -1;
    is_moving_                      = -1;
    serial_debug_                   = 0;
}

bool LiftController::setDefaults()
{
    bool all_success = true;
    all_success = setMinMaxLiftPosition(LIFT_CONTROLLER_MIN_LIFT_POSITION + 1, LIFT_CONTROLLER_MAX_LIFT_POSITION) && all_success;
    all_success = setMinMaxLiftSpeed(LIFT_CONTROLLER_MIN_LIFT_SPEED, LIFT_CONTROLLER_MAX_LIFT_SPEED) && all_success;
    return all_success;
}

void LiftController::showState()
{
    ROS_INFO_NAMED(ROS_NAME, "Enabled                       : %d", controller_enabled_);
    ROS_INFO_NAMED(ROS_NAME, "Set lift speed                : %d", set_lift_speed_);
    ROS_INFO_NAMED(ROS_NAME, "Set lift speed                : %d%%", set_lift_speed_percentage_);
    ROS_INFO_NAMED(ROS_NAME, "Set lift position             : %d", set_lift_position_);
    ROS_INFO_NAMED(ROS_NAME, "Set lift position             : %d%%", set_lift_position_percentage_);

    ROS_INFO_NAMED(ROS_NAME, "Bumper states                 : %d|%d|%d|%d|%d|%d|%d|%d"  , bumper_states_[0]
                                                                                        , bumper_states_[1]
                                                                                        , bumper_states_[2]
                                                                                        , bumper_states_[3]
                                                                                        , bumper_states_[4]
                                                                                        , bumper_states_[5]
                                                                                        , bumper_states_[6]
                                                                                        , bumper_states_[7]);
    ROS_INFO_NAMED(ROS_NAME, "Lift Position                 : %d", cur_position_);
    ROS_INFO_NAMED(ROS_NAME, "Lift Position                 : %d%%", cur_position_percentage_);
    ROS_INFO_NAMED(ROS_NAME, "Lift Position Error           : %d", cur_position_error_);
    ROS_INFO_NAMED(ROS_NAME, "Is in postition               : %d", is_in_position_);
    ROS_INFO_NAMED(ROS_NAME, "Is moving                     : %d", is_moving_);
    ROS_INFO_NAMED(ROS_NAME, "MIN/MAX position              : %d/%d", min_lift_position_, max_lift_position_);
    ROS_INFO_NAMED(ROS_NAME, "MIN/MAX speed                 : %d/%d", min_lift_speed_, max_lift_speed_);
    ROS_INFO_NAMED(ROS_NAME, "Safety state                  : %d|%d|%d|%d|%d|%d"    , safety_state_[0]
                                                                                    , safety_state_[1]
                                                                                    , safety_state_[2]
                                                                                    , safety_state_[3]
                                                                                    , safety_state_[4]
                                                                                    , safety_state_[5]);
    ROS_INFO_NAMED(ROS_NAME, "Safety Input State            : %d", safety_input_state_);
    ROS_INFO_NAMED(ROS_NAME, "Extra Input State             : %d", extra_input_state_);
    ROS_INFO_NAMED(ROS_NAME, "Extra Output State            : %d", extra_output_state_);

    for(int i = 0; i < 8; i++)
        ROS_INFO_NAMED(ROS_NAME, "ADC(%d) CUR/AVG/MIN/MAX   : %d, %d, %d, %d", i, adc_eng_[i][0], adc_eng_[i][1], adc_eng_[i][2], adc_eng_[i][3]);
}

bool LiftController::update()
{
    if(!controller_enabled_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Trying to update lift while not enabled.");
        return false;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Updating lift and bumper controller state."); 
    
    bool all_success = true;
    all_success = getBumperStates() && all_success;
    all_success = getSafetyInput() && all_success;
    all_success = getSafetyState() && all_success;
    all_success = getExtraInput() && all_success;
    all_success = getCurrentPosition() && all_success;
    all_success = getCurrentError() && all_success;
    all_success = getPositionPercentage() && all_success;
    all_success = getIsInPosition() && all_success;
    all_success = getIsMoving() && all_success;
    all_success = getSerialDebug() && all_success;
    // all_success = getADCRAW() && all_success;            // Disabled to increase update speed
    // all_success = getADCVOL() && all_success;
    // all_success = getADCENG() && all_success;
    all_success = getSetPosition() && all_success;
    all_success = getSetPositionPercentage() && all_success;
    all_success = getSetSpeed() && all_success;
    all_success = getSetSpeedPercentage() && all_success;
    // all_success = getADCINT() && all_success; //! @todo OH: 

    handleSafety(); //! @todo OH: HACK should this be in update?

    return all_success;
}

//! @todo OH: HACK
void LiftController::handleSafety()
{
    if(safety_input_state_ == 1 or safety_state_[0] == 1)
    {
        ROS_WARN_NAMED(ROS_NAME, "Emergency stop pressed, wheels will be powered down.");
        sh_emergency_ = true;
    }
}

bool LiftController::getRequestedEnabledState()
{
    return requested_controller_enabled_; 
}

bool LiftController::setPose(int speed_precentage, int position_percentage)
{
    if(!controller_enabled_)
    {
        ROS_ERROR_NAMED(ROS_NAME, "Cannot set pose while lift is disabled.");
        return false;
    }

    if(speed_precentage < 0 || speed_precentage > 100 || position_percentage < 0 || position_percentage > 100)
    {
        ROS_ERROR_NAMED(ROS_NAME, "Invalid percentages requested, position %d%%, speed %d%%.", position_percentage, speed_precentage);
        return false;
    } 

    ROS_INFO_NAMED(ROS_NAME, "Moving lift to %d%% at %d%% of maximal speed.", position_percentage, speed_precentage);       

    ControllerResponse response(LIFT_CONTROLLER_MOVE_TO, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(speed_precentage, set_lift_speed_percentage_, "Speed was not set to requested value."));
    response.addExpectedDataItem(ControllerData(position_percentage, set_lift_position_percentage_, "Position was not set to requested value."));
    ControllerCommand  command(LIFT_CONTROLLER_MOVE_TO, response);
    command.addDataItem(speed_precentage);
    command.addDataItem(position_percentage);

    return executeCommand(command);
}

int LiftController::getPose()
{
    return cur_position_percentage_;
}

bool LiftController::forceMotorStop()
{
    return simpleCommand(LIFT_CONTROLLER_FORCE_STOP, HARDWARE_CONTROL_TIMEOUT);
}

bool LiftController::setSerialDebug(bool state)
{
     if(state == true)
        return setValue(LIFT_CONTROLLER_SERIAL_DEBUG_MODE, HARDWARE_CONTROL_TIMEOUT, 1, serial_debug_);
    else
        return setValue(LIFT_CONTROLLER_SERIAL_DEBUG_MODE, HARDWARE_CONTROL_TIMEOUT, 0, serial_debug_);
}

bool LiftController::resetADCMinMax(int adc_number)
{
    return simpleCommand(LIFT_CONTROLLER_RESET_SPECIFIC_ADC, HARDWARE_CONTROL_TIMEOUT, adc_number);
}

bool LiftController::resetAllADCMinMax()
{
    return simpleCommand(LIFT_CONTROLLER_RESET_ALL_ADC, HARDWARE_CONTROL_TIMEOUT);
}

bool LiftController::reset()
{
    if(disable())
        return enable();

    return false;
}

bool LiftController::resetComm()
{
    stopWatchdog();
    stopReadloop();
    spawnReadloop();
    bool success = simpleCommand(LIFT_CONTROLLER_RESET_COMM, HARDWARE_CONTROL_RESET_COMM_TIMEOUT);    
    spawnWatchdog();
    return success;
}

bool LiftController::resetAlarm()
{
    return simpleCommand(LIFT_CONTROLLER_RESET_ALARM_STATE, HARDWARE_CONTROL_TIMEOUT);
}

bool LiftController::getBumperStates()
{
    ControllerResponse response(LIFT_CONTROLLER_GET_BUMPERS, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 8; i++)
        response.addExpectedDataItem(ControllerData(bumper_states_[i]));

    ControllerCommand  command(LIFT_CONTROLLER_GET_BUMPERS, response);

    return executeCommand(command);
}

bool LiftController::getSafetyInput()
{
    return getValue(LIFT_CONTROLLER_GET_SAFETY_INPUT, HARDWARE_CONTROL_TIMEOUT, safety_input_state_);  
}

bool LiftController::getSafetyState()
{
    ControllerResponse response(LIFT_CONTROLLER_GET_SAFETY_STATE, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 6; i++)
        response.addExpectedDataItem(ControllerData(safety_state_[i]));

    ControllerCommand  command(LIFT_CONTROLLER_GET_SAFETY_STATE, response);

    return executeCommand(command);
}

bool LiftController::getExtraInput()
{
    return getValue(LIFT_CONTROLLER_GET_EXTRA_INPUT, HARDWARE_CONTROL_TIMEOUT, extra_input_state_);
}

bool LiftController::getCurrentPosition()
{
    return getValue(LIFT_CONTROLLER_GET_POS, HARDWARE_CONTROL_TIMEOUT, cur_position_);
}

bool LiftController::getCurrentError()
{
    return getValue(LIFT_CONTROLLER_GET_POS_ERROR, HARDWARE_CONTROL_TIMEOUT, cur_position_error_);
}

bool LiftController::getPositionPercentage()
{
    return getValue(LIFT_CONTROLLER_GET_POS_PERCENTAGE, HARDWARE_CONTROL_TIMEOUT, cur_position_percentage_); 
}

bool LiftController::getIsInPosition()
{
    return getValue(LIFT_CONTROLLER_GET_IS_IN_POSITION, HARDWARE_CONTROL_TIMEOUT, is_in_position_);
}

bool LiftController::getIsMoving()
{
    return getValue(LIFT_CONTROLLER_GET_IS_MOVING, HARDWARE_CONTROL_TIMEOUT, is_moving_);
}

bool LiftController::getSerialDebug()
{
    return getValue(LIFT_CONTROLLER_GET_SERIAL_DEBUG_MODE, HARDWARE_CONTROL_TIMEOUT, serial_debug_);
}

//! @todo OH: Make ADC class?
bool LiftController::getADCRAW()
{
    ControllerResponse response(LIFT_CONTROLLER_GET_ADC_RAW, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 8; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_raw_[i][0]));
        response.addExpectedDataItem(ControllerData(adc_raw_[i][1]));
        response.addExpectedDataItem(ControllerData(adc_raw_[i][2]));
        response.addExpectedDataItem(ControllerData(adc_raw_[i][3]));
    }
    ControllerCommand  command(LIFT_CONTROLLER_GET_ADC_RAW, response);

    return executeCommand(command);
}

bool LiftController::getADCVOL()
{
    ControllerResponse response(LIFT_CONTROLLER_GET_ADC_VOL, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 8; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_vol_[i][0]));
        response.addExpectedDataItem(ControllerData(adc_vol_[i][1]));
        response.addExpectedDataItem(ControllerData(adc_vol_[i][2]));
        response.addExpectedDataItem(ControllerData(adc_vol_[i][3]));
    }
    ControllerCommand  command(LIFT_CONTROLLER_GET_ADC_VOL, response);

    return executeCommand(command);
}

bool LiftController::getADCENG()
{
    ControllerResponse response(LIFT_CONTROLLER_GET_ADC_ENG, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 8; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_eng_[i][0]));
        response.addExpectedDataItem(ControllerData(adc_eng_[i][1]));
        response.addExpectedDataItem(ControllerData(adc_eng_[i][2]));
        response.addExpectedDataItem(ControllerData(adc_eng_[i][3]));
    }
    ControllerCommand  command(LIFT_CONTROLLER_GET_ADC_ENG, response);

    return executeCommand(command);
}

bool LiftController::getADCINT()
{
    ControllerResponse response(LIFT_CONTROLLER_GET_ADC_INT, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 8; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_int_[i][0]));
        response.addExpectedDataItem(ControllerData(adc_int_[i][1]));
    }
    ControllerCommand  command(LIFT_CONTROLLER_GET_ADC_INT, response);

    return executeCommand(command);
}

bool LiftController::getSetPosition()
{
    return getValue(LIFT_CONTROLLER_GET_SET_POS, HARDWARE_CONTROL_TIMEOUT, set_lift_position_);
}

bool LiftController::getSetPositionPercentage()
{
    return getValue(LIFT_CONTROLLER_GET_SET_POS_PERCENTAGE, HARDWARE_CONTROL_TIMEOUT, set_lift_position_percentage_);
}

bool LiftController::getSetSpeed()
{
    return getValue(LIFT_CONTROLLER_GET_SET_SPEED, HARDWARE_CONTROL_TIMEOUT, set_lift_speed_);
}

bool LiftController::getSetSpeedPercentage()
{
    return getValue(LIFT_CONTROLLER_GET_SET_SPEED_PERCENTAGE, HARDWARE_CONTROL_TIMEOUT, set_lift_speed_percentage_);
}

// === SETTERS === 

bool LiftController::setSafetyOutput(bool state)
{
    if(state == true)
        return setValue(LIFT_CONTROLLER_SET_SAFETY_OUTPUT, HARDWARE_CONTROL_TIMEOUT, 1, safety_output_state_);
    else
        return setValue(LIFT_CONTROLLER_SET_SAFETY_OUTPUT, HARDWARE_CONTROL_TIMEOUT, 0, safety_output_state_);
}

bool LiftController::setExtraOutput(bool state)
{
     if(state == true)
        return setValue(LIFT_CONTROLLER_SET_EXTRA_OUTPUT, HARDWARE_CONTROL_TIMEOUT, 1, extra_output_state_);
    else
        return setValue(LIFT_CONTROLLER_SET_EXTRA_OUTPUT, HARDWARE_CONTROL_TIMEOUT, 0, extra_output_state_);
}

bool LiftController::setMinMaxLiftPosition(int min_position, int max_position)
{
    ControllerResponse response(LIFT_CONTROLLER_SET_MINMAX_MOTOR_POS, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(min_position, min_lift_position_, "Setting minimal motor position unsuccessfull."));
    response.addExpectedDataItem(ControllerData(max_position, max_lift_position_, "Setting maximal motor position unsuccessfull."));
    ControllerCommand  command(LIFT_CONTROLLER_SET_MINMAX_MOTOR_POS, response);
    command.addDataItem(min_position);
    command.addDataItem(max_position);

    return executeCommand(command);
}

bool LiftController::setMinMaxLiftSpeed(int min_speed, int max_speed)
{
    ControllerResponse response(LIFT_CONTROLLER_SET_MINMAX_MOTOR_SPEED, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(min_speed, min_lift_speed_, "Setting minimal motor speed unsuccessfull."));
    response.addExpectedDataItem(ControllerData(max_speed, max_lift_speed_, "Setting maximal motor speed unsuccessfull."));
    ControllerCommand  command(LIFT_CONTROLLER_SET_MINMAX_MOTOR_SPEED, response);
    command.addDataItem(min_speed);
    command.addDataItem(max_speed);

    return executeCommand(command);
}

void LiftController::CB_SetControllerState(const std_msgs::Bool::ConstPtr& enable_message)
{
    ROS_INFO_NAMED(ROS_NAME.c_str(), "%s lift", enable_message->data?"Enabling":"Disabling");

    requested_controller_enabled_ = enable_message->data;
    if(enable_message->data) 
    {   
        if(controller_enabled_)
            ROS_INFO_NAMED(ROS_NAME, "Lift already enabled.");
        else if(enable())
            ROS_INFO_NAMED(ROS_NAME, "Lift successfully enabled.");
        else
            ROS_WARN_NAMED(ROS_NAME, "Lift NOT enabled!");
    }
    else
    {
        if(!controller_enabled_)
            ROS_INFO_NAMED(ROS_NAME, "Lift already disabled.");
        else if(disable())
            ROS_INFO_NAMED(ROS_NAME, "Lift successfully disabled");
        else
            ROS_WARN_NAMED(ROS_NAME, "Lift NOT disabled!");
    }          
}

void LiftController::CB_LiftPositionRequest(const rose21_platform::lift_command::ConstPtr& lift_command)
{
    setPose(lift_command->speed_percentage, 100.0 - lift_command->position_percentage); 
}
