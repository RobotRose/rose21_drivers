/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Okke Hendriks
*    Date  : 2014/10/13
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/

#include <rose21_power_controller/power_controller.hpp>


PowerController::PowerController(string name, ros::NodeHandle n, string serial_port, int baudrate)
    : HardwareController<Serial>()
    , sh_power_output_states_(SharedVariable<std::vector<int>>("power_controller_output_states"))
    , sh_emergency_(SharedVariable<bool>("emergency"))
{
    n_                      = n;
    name_                   = name;
    comm_interface_         = Serial(name, serial_port, baudrate);

    selected_battery_pub_       = n_.advertise<std_msgs::Int32>(name_ + "/selected_battery", 1, true);
    battery1_state_pub_         = n_.advertise<rose_base_msgs::bumpers_state>(name_ + "/battery1/state", 1, true);
    battery2_state_pub_         = n_.advertise<rose_base_msgs::bumpers_state>(name_ + "/battery2/state", 1, true);
    battery_combined_state_pub_ = n_.advertise<rose_base_msgs::bumpers_state>(name_ + "/battery_combined/state", 1, true);

    bat_warn_time_ = ros::Time::now();

    // Host mutable, published std::vector<int>
    sh_power_output_states_.host(false, true, ros::Rate(1));

    // Connect to state of emercency
    sh_emergency_.connect(ros::Duration(0.1));
    
    bool default_output_states[POWERCONTROLLER_NR_OUTPUTS] = DEFAULT_OUTPUT_STATES;
    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++ )
    {
        sh_power_output_states_.push_back(-1);
    }

    resetState();

    ROS_INFO_NAMED(ROS_NAME, "Started %s", name_.c_str());
}

PowerController::~PowerController()
{
    ROS_INFO_NAMED(ROS_NAME, "Stopped %s", name_.c_str());
}

bool PowerController::enable()
{
    ROS_INFO_NAMED(ROS_NAME, "Enabling power controller.");
    if(!get_comm_interface()->connect())   
        return false;

    // Also starts comm
    resetComm();

    if(!checkControllerID(POWER_CONTROL_FIRMWARE_ID)) 
        return false;

    if(!checkFirmwareVersion(POWER_CONTROL_FIRMWARE_MAJOR_VERSION, POWER_CONTROL_FIRMWARE_MINOR_VERSION))
        return false;

    ROS_INFO_NAMED(ROS_NAME, "Power controller with firmware version %d.%d connected.", received_firmware_major_version_, received_firmware_minor_version_);

    
    if(!setDefaults())
    {
        ROS_WARN_NAMED(ROS_NAME, "Unable to set default values.");
        return false;
    }
    else
    {
        ROS_INFO_NAMED(ROS_NAME, "Default parameters set.");
    }

    enabled_ = true;

    // Run an initial update
    if(!update())
        return false;

    // Set outputs to the default state
    bool default_output_states[POWERCONTROLLER_NR_OUTPUTS] = DEFAULT_OUTPUT_STATES;
    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++ )
    {
        if(default_output_states[i])
            setPowerOutput(i, 1);
        else
            setPowerOutput(i, 0);
    }
    
    return enabled_;
}

bool PowerController::setDefaults()
{
    bool all_success = true;
    all_success = setAlarmSound(LOW_BATTERY_SOUND_ENABLED) && all_success;
    all_success = setAlarmSoundInterval(LOW_BATTERY_BEEP_INTERVAL) && all_success;
    all_success = setSwitchSound(SWITCH_SOUND_ENABLED) && all_success;
    all_success = setMinimalVoltage(MINIMAL_BATTERY_VOLTAGE) && all_success;
    all_success = setWarningVoltage(WARNING_BATTERY_VOLTAGE) && all_success;
    all_success = setAutoSwitchMode(AUTO_SWITCH_MODE_ENABLED) && all_success;
    all_success = setAutoSwitchVoltage(AUTO_SWITCH_BATTERY_VOLTAGE) && all_success;

    // Read the currently set default outputs state at the controller
    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++)
        all_success = getDefaultOutputState(i) && all_success;

    // Check if this default output state is the same as defined as the default parameters in the header file
    // If they ar not the same, write the correct default state to the EEPROM of the controller
    bool default_output_states[POWERCONTROLLER_NR_OUTPUTS] = DEFAULT_OUTPUT_STATES;
    ROS_INFO_NAMED(ROS_NAME, "Setting default output states:");
    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++ )
    {
        if(default_output_states[i] != set_default_outputs_[i])
        {
            all_success = setDefaultOutputState(i, default_output_states[i]) && all_success;
            ROS_INFO_NAMED(ROS_NAME, " Setting default output %d -> %d", i, default_output_states[i]);
        }
        else
            ROS_INFO_NAMED(ROS_NAME, " Default output %d = %d", i, default_output_states[i]);
    }

    return all_success;
}

void PowerController::resetState()
{
    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++)
        sh_power_output_states_[i] = 0;

    selected_battery            = -1;
    switch_voltage              = -1;
    warning_voltage             = -1;
    minimal_voltage             = -1;
    safety_input_state          = 0;
    extra_input_state           = 0;
    extra_output_state          = 0;
    // float   adc_raw[16][4];
    // float   adc_vol[16][4];
    // float   adc_eng[16][4];
    // float   adc_int[16][2];

    battery_auto_mode           = 1;
    battery_switch_sound        = 1;
    battery_voltage_raw[0]      = -1;
    battery_voltage_raw[1]      = -1;
    battery_voltage_avg[0]      = -1;
    battery_voltage_avg[1]      = -1;
    alarm_sound                 = 1;
    alarm_sound_interval        = 5000;
    watchdog_treshold           = 1000;
    serial_debug_mode           = 0;

    shutdown_timer_             = 0;
    restart_timer_              = 0;
}


bool PowerController::update()
{
    if(!enabled_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Enable before updating power controller status.");
        return false;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Updating power controller status.");

    bool all_success = true;
    all_success = getPowerOutputs() && all_success;
    all_success = getSelectedBattery() && all_success;
    all_success = getSwitchVoltage() && all_success;
    all_success = getWarningVoltage() && all_success;
    all_success = getMinimalVoltage() && all_success;
    all_success = getSafetyInput() && all_success;
    all_success = getExtraInput() && all_success;
    all_success = getBatteryVoltageRaw() && all_success;
    all_success = getBatteryVoltageAverage() && all_success;
    all_success = getSwitchSoundState() && all_success;
    all_success = getAlarmSoundState() && all_success;
    all_success = getAlarmSoundInterval() && all_success;
    all_success = getWatchdogTreshold() && all_success;
    all_success = getShutdownTime() && all_success;
    all_success = getRestartTime() && all_success;

    if(sh_emergency_)
    {
        if(getPowerOutput(3) == true || getPowerOutput(4) == true)
        {
            ROS_DEBUG_NAMED(ROS_NAME, "Emergency, turning off power to wheelunits.");
            setPowerOutput(3, false);
            setPowerOutput(4, false);
        }
    }
    else
    {
        if(getPowerOutput(3) == false || getPowerOutput(4) == false)
        {
            ROS_DEBUG_NAMED(ROS_NAME, "Emergency resolved, turning on power to wheelunits.");
            setPowerOutput(3, true);
            setPowerOutput(4, true);
        }
    }

    if(getCombinedBatteryState().percentage < GUI_BAT_WARNING_PRECENTAGE and ros::Time::now().toSec() - bat_warn_time_.toSec() > GUI_BAT_WARNING_INTERVAL) 
    {
        bat_warn_time_ = ros::Time::now();
        operator_gui_.warn("Batterij bijna leeg, laad batterij op.");
    }

    return all_success;
}

bool PowerController::completeUpdate()
{
    if(!enabled_)
    {
        ROS_WARN_NAMED(ROS_NAME, "Enable before updating power controller status.");
        return false;
    }

    bool all_success = true;
    all_success = update() && all_success;
    all_success = getADCRAW() && all_success;
    all_success = getADCVOL() && all_success;
    all_success = getADCENG() && all_success;
    all_success = getADCINT() && all_success;

    return all_success;
}

rose_base_msgs::bumpers_state PowerController::getBatteryState(int battery_number)
{
    int index                       = battery_number - 1;
    rose_base_msgs::bumpers_state battery_state;
    battery_state.id                = battery_number;
    battery_state.empty_voltage     = ((float)minimal_voltage)/1000.0;
    battery_state.current_voltage   = ((float)battery_voltage_avg[index])/1000.0;
    battery_state.full_voltage      = ((float)POWER_CONTROLLER_BATTERY_FULL_VOLTAGE)/1000.0;
    float percentage                = (battery_state.current_voltage - battery_state.empty_voltage)/(battery_state.full_voltage - battery_state.empty_voltage) * 100.0;
    battery_state.percentage        = std::fmin(100.0, std::fmax(0.0, percentage));
    return battery_state;
}

rose_base_msgs::bumpers_state PowerController::getCombinedBatteryState()
{
    rose_base_msgs::bumpers_state battery_combined_state;
    rose_base_msgs::bumpers_state bat1 = getBatteryState(1);
    rose_base_msgs::bumpers_state bat2 = getBatteryState(2);

    battery_combined_state.id                   = -1;
    battery_combined_state.empty_voltage        = fmin(bat1.empty_voltage, bat2.empty_voltage);
    
    // Calculate average combined battery voltage, disregard voltages below minimal voltage
    std::vector<float> averaging_voltages;
    if(bat1.current_voltage > bat1.empty_voltage)
        averaging_voltages.push_back(bat1.current_voltage);
    if(bat2.current_voltage > bat2.empty_voltage)
        averaging_voltages.push_back(bat2.current_voltage);

    for(const auto& voltage : averaging_voltages)
        battery_combined_state.current_voltage += voltage;   
    battery_combined_state.current_voltage      /= averaging_voltages.size();

    battery_combined_state.full_voltage         = fmax(bat1.full_voltage, bat2.full_voltage);
    float percentage                            = (battery_combined_state.current_voltage - battery_combined_state.empty_voltage)/(battery_combined_state.full_voltage - battery_combined_state.empty_voltage) * 100.0;
    battery_combined_state.percentage           = std::fmin(100.0, std::fmax(0.0, percentage));
    return battery_combined_state;
}

void PowerController::publishBatteryStates()
{
    std_msgs::Int32 selected_battery_msg;
    selected_battery_msg.data = selected_battery;
    selected_battery_pub_.publish(selected_battery_msg);
    
    battery1_state_pub_.publish(getBatteryState(1));
    battery2_state_pub_.publish(getBatteryState(2));
    battery_combined_state_pub_.publish(getCombinedBatteryState());
}

void PowerController::showState()
{
    ROS_DEBUG_NAMED(ROS_NAME, "Selected Battery              : %d", selected_battery);
    ROS_DEBUG_NAMED(ROS_NAME, "Auto battery select           : %d", battery_auto_mode);
    ROS_DEBUG_NAMED(ROS_NAME, "Low voltage sound interval    : %d", alarm_sound_interval);
    ROS_DEBUG_NAMED(ROS_NAME, "Switch sound                  : %d", battery_switch_sound);
    ROS_DEBUG_NAMED(ROS_NAME, "Bat 1 raw | avg voltage       : %dmV | %dmV", battery_voltage_raw[0], battery_voltage_avg[0]);
    ROS_DEBUG_NAMED(ROS_NAME, "Bat 2 raw | avg voltage       : %dmV | %dmV", battery_voltage_raw[1], battery_voltage_avg[1]);    
    ROS_DEBUG_NAMED(ROS_NAME, "Output states                 : %d|%d|%d|%d|%d|%d", (int)sh_power_output_states_[0]
                                                                                , (int)sh_power_output_states_[1]
                                                                                , (int)sh_power_output_states_[2]
                                                                                , (int)sh_power_output_states_[3]
                                                                                , (int)sh_power_output_states_[4]
                                                                                , (int)sh_power_output_states_[5]);    
    ROS_DEBUG_NAMED(ROS_NAME, "Default output states         : %d|%d|%d|%d|%d|%d", set_default_outputs_[0]
                                                                                , set_default_outputs_[1]
                                                                                , set_default_outputs_[2]
                                                                                , set_default_outputs_[3]
                                                                                , set_default_outputs_[4]
                                                                                , set_default_outputs_[5]);
    ROS_DEBUG_NAMED(ROS_NAME, "Switch voltage                : %dmV", switch_voltage);
    ROS_DEBUG_NAMED(ROS_NAME, "Warning voltage               : %dmV", warning_voltage);
    ROS_DEBUG_NAMED(ROS_NAME, "Minimal voltage               : %dmV", minimal_voltage);
    ROS_DEBUG_NAMED(ROS_NAME, "Safety Input State            : %d", safety_input_state);
    ROS_DEBUG_NAMED(ROS_NAME, "Extra Input State             : %d", extra_input_state);
    ROS_DEBUG_NAMED(ROS_NAME, "Extra Output State            : %d", extra_output_state);

    int set_timer_value[6];
    int current_timer_value[6];
    for(int i = 0; i < 6; i++)
    {
        getSetTimerValue(i, set_timer_value[i]);
        getCurrentTimerValue(i, current_timer_value[i]);
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Timer Set values              : %6d|%6d|%6d|%6d|%6d|%6d"  , set_timer_value[0]
                                                                                        , set_timer_value[1]
                                                                                        , set_timer_value[2]
                                                                                        , set_timer_value[3]
                                                                                        , set_timer_value[4]
                                                                                        , set_timer_value[5]);
    ROS_DEBUG_NAMED(ROS_NAME, "Timer current values          : %6d|%6d|%6d|%6d|%6d|%6d"  , current_timer_value[0]
                                                                                        , current_timer_value[1]
                                                                                        , current_timer_value[2]
                                                                                        , current_timer_value[3]
                                                                                        , current_timer_value[4]
                                                                                        , current_timer_value[5]);


    for(int i = 0; i < 16; i++)
        ROS_DEBUG_NAMED(ROS_NAME, "ADC(%d) CUR/AVG/MIN/MAX   : %d, %d, %d, %d", i, adc_eng[i][0], adc_eng[i][1], adc_eng[i][2], adc_eng[i][3]);
}

void PowerController::togglePowerOutput(int index)
{
    if(index < 0 || index > 5)
        return;

    if(sh_power_output_states_[index] == 0)
        setPowerOutput(index, 1);
    else
        setPowerOutput(index, 0);
}

bool PowerController::increaseSoundInterval()
{
    return setAlarmSoundInterval(alarm_sound_interval + 1000);
}

bool PowerController::decreaseSoundInterval()
{
    return setAlarmSoundInterval(alarm_sound_interval - 1000);
}

// Getters
bool PowerController::getPowerOutputs()
{
    int state[POWERCONTROLLER_NR_OUTPUTS];
    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++)
        state[i] = (int)sh_power_output_states_[i];
    
    ControllerResponse response(POWER_CONTROL_GET_POWER_OUTPUTS, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++)
        response.addExpectedDataItem(ControllerData(state[i]));

    ControllerCommand  command(POWER_CONTROL_GET_POWER_OUTPUTS, response);

    if(!executeCommand(command))
        return false;

    for(int i = 0; i < POWERCONTROLLER_NR_OUTPUTS; i++)
        sh_power_output_states_[i] = (bool)state[i];
    
    return true;
}

bool PowerController::getSelectedBattery()
{
    return getValue(POWER_CONTROL_GET_BATTERY, HARDWARE_CONTROL_TIMEOUT, selected_battery);
}

bool PowerController::getSwitchVoltage()
{
    return getValue(POWER_CONTROL_GET_SWITCH_VIN, HARDWARE_CONTROL_TIMEOUT, switch_voltage);
}

bool PowerController::getWarningVoltage()
{
    return getValue(POWER_CONTROL_GET_WARNING_VIN, HARDWARE_CONTROL_TIMEOUT, warning_voltage);
}

bool PowerController::getMinimalVoltage()
{
    return getValue(POWER_CONTROL_GET_MINIMAL_VIN, HARDWARE_CONTROL_TIMEOUT, minimal_voltage);
}

bool PowerController::getSafetyInput()
{
    return getValue(POWER_CONTROL_GET_SAFETY_IN, HARDWARE_CONTROL_TIMEOUT, safety_input_state);
}

bool PowerController::getExtraInput()
{
    return getValue(POWER_CONTROL_GET_EXTRA_IN, HARDWARE_CONTROL_TIMEOUT, extra_input_state);
}

bool PowerController::getADCRAW()
{
    ControllerResponse response(POWER_CONTROL_GET_ADC_RAW, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 16; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_raw[i][0]));
        response.addExpectedDataItem(ControllerData(adc_raw[i][1]));
        response.addExpectedDataItem(ControllerData(adc_raw[i][2]));
        response.addExpectedDataItem(ControllerData(adc_raw[i][3]));
    }
    ControllerCommand  command(POWER_CONTROL_GET_ADC_RAW, response);

    return executeCommand(command);
}

bool PowerController::getADCVOL()
{
    ControllerResponse response(POWER_CONTROL_GET_ADC_VOL, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 16; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_vol[i][0]));
        response.addExpectedDataItem(ControllerData(adc_vol[i][1]));
        response.addExpectedDataItem(ControllerData(adc_vol[i][2]));
        response.addExpectedDataItem(ControllerData(adc_vol[i][3]));
    }
    ControllerCommand  command(POWER_CONTROL_GET_ADC_VOL, response);

    return executeCommand(command);
}

bool PowerController::getADCENG()
{
    ControllerResponse response(POWER_CONTROL_GET_ADC_ENG, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 16; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_eng[i][0]));
        response.addExpectedDataItem(ControllerData(adc_eng[i][1]));
        response.addExpectedDataItem(ControllerData(adc_eng[i][2]));
        response.addExpectedDataItem(ControllerData(adc_eng[i][3]));
    }
    ControllerCommand  command(POWER_CONTROL_GET_ADC_ENG, response);

    return executeCommand(command);
}

bool PowerController::getADCINT()
{
    ControllerResponse response(POWER_CONTROL_GET_ADC_INT, HARDWARE_CONTROL_TIMEOUT);
    for(int i = 0; i < 16; i++)
    {
        response.addExpectedDataItem(ControllerData(adc_int[i][0]));
        response.addExpectedDataItem(ControllerData(adc_int[i][1]));
    }
    ControllerCommand  command(POWER_CONTROL_GET_ADC_INT, response);

    return executeCommand(command);
}

bool PowerController::getBatteryVoltageRaw()
{
    ControllerResponse response(POWER_CONTROL_GET_BAT_VOLT_RAW, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(battery_voltage_raw[0]));
    response.addExpectedDataItem(ControllerData(battery_voltage_raw[1]));
    ControllerCommand  command(POWER_CONTROL_GET_BAT_VOLT_RAW, response);

    return executeCommand(command);
}

bool PowerController::getBatteryVoltageAverage()
{
    ControllerResponse response(POWER_CONTROL_GET_BAT_VOLT_AVG, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(battery_voltage_avg[0]));
    response.addExpectedDataItem(ControllerData(battery_voltage_avg[1]));
    ControllerCommand  command(POWER_CONTROL_GET_BAT_VOLT_AVG, response);

    return executeCommand(command);
}

bool PowerController::getSwitchSoundState()
{
    return getValue(POWER_CONTROL_GET_SWITCH_SOUND, HARDWARE_CONTROL_TIMEOUT, battery_switch_sound);
}

bool PowerController::getAlarmSoundState()
{
    return getValue(POWER_CONTROL_GET_ALARM_SOUND, HARDWARE_CONTROL_TIMEOUT, alarm_sound);
}

bool PowerController::getAlarmSoundInterval()
{
    return getValue(POWER_CONTROL_GET_ALARM_SOUND_INTER, HARDWARE_CONTROL_TIMEOUT, alarm_sound_interval);
}

bool PowerController::getShutdownTime()
{
    return getValue(POWER_CONTROL_GET_CUR_TIMER_VALUE, HARDWARE_CONTROL_TIMEOUT, SHUTDOWN_TIMER_ID, shutdown_timer_);
}

bool PowerController::getRestartTime()
{
    return getValue(POWER_CONTROL_GET_CUR_TIMER_VALUE, HARDWARE_CONTROL_TIMEOUT, RESTART_TIMER_ID, restart_timer_);
}

bool PowerController::getSetTimerValue(int timer_id, int& timer_set_value)
{
    return getValue(POWER_CONTROL_GET_SET_TIMER_VALUE, HARDWARE_CONTROL_TIMEOUT, timer_id, timer_set_value);
}

bool PowerController::getCurrentTimerValue(int timer_id, int& timer_current_value)
{
    return getValue(POWER_CONTROL_GET_CUR_TIMER_VALUE, HARDWARE_CONTROL_TIMEOUT, timer_id, timer_current_value);
}

// Setters
bool PowerController::setAutoSwitchVoltage(int mv)
{
    return setValue(POWER_CONTROL_SET_AUTO_SWITCH_VIN, HARDWARE_CONTROL_TIMEOUT, (int)(mv), switch_voltage);
}

bool PowerController::setWarningVoltage(int mv)
{
    return setValue(POWER_CONTROL_SET_WARNING_VIN, HARDWARE_CONTROL_TIMEOUT, (int)(mv), warning_voltage);
}

bool PowerController::setMinimalVoltage(int mv)
{
    return setValue(POWER_CONTROL_SET_MINIMAL_VIN, HARDWARE_CONTROL_TIMEOUT, (int)(mv), minimal_voltage);
}

bool PowerController::setExtraOutput(bool state)
{
    if(state == true)
        return setValue(POWER_CONTROL_SET_EXTRA_OUT, HARDWARE_CONTROL_TIMEOUT, 1);
    else
        return setValue(POWER_CONTROL_SET_EXTRA_OUT, HARDWARE_CONTROL_TIMEOUT, 0);
}

bool PowerController::setAutoSwitchMode(bool state)
{
    if(state == true)
        return setValue(POWER_CONTROL_SET_AUTO_SWITCH_MODE, HARDWARE_CONTROL_TIMEOUT, 1, battery_auto_mode);
    else
        return setValue(POWER_CONTROL_SET_AUTO_SWITCH_MODE, HARDWARE_CONTROL_TIMEOUT, 0, battery_auto_mode);
}

bool PowerController::setPowerOutput(int index, int state)
{
    // Check if already in correct state
    if(sh_power_output_states_[index] == state)
        return true;

    ControllerResponse response(POWER_CONTROL_SET_POWER_OUTPUT, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(index));
    response.addExpectedDataItem(ControllerData(state));

    ControllerCommand  command(POWER_CONTROL_SET_POWER_OUTPUT, response);
    command.addDataItem(index);
    command.addDataItem(state);

    if(!executeCommand(command))
        return false;

    sh_power_output_states_[index] = state;

    return true;
}

bool PowerController::getPowerOutput(int index)
{
    if(sh_power_output_states_[index] == 1)
        return true;
    else
        return false;
}

bool PowerController::setSwitchSound(bool state)
{
    if(state == true)
        return setValue(POWER_CONTROL_SET_SWITCH_SOUND, HARDWARE_CONTROL_TIMEOUT, 1, battery_switch_sound);
    else
        return setValue(POWER_CONTROL_SET_SWITCH_SOUND, HARDWARE_CONTROL_TIMEOUT, 0, battery_switch_sound);
}

bool PowerController::setAlarmSound(bool state)
{
    if(state == true)
        return setValue(POWER_CONTROL_SET_ALARM_SOUND, HARDWARE_CONTROL_TIMEOUT, 1, alarm_sound);
    else
        return setValue(POWER_CONTROL_SET_ALARM_SOUND, HARDWARE_CONTROL_TIMEOUT, 0, alarm_sound);
}

bool PowerController::setAlarmSoundInterval(int interval)
{
    return setValue(POWER_CONTROL_SET_ALARM_INTERVAL, HARDWARE_CONTROL_TIMEOUT, interval, alarm_sound_interval);
}    

// Other commands
bool PowerController::resetComm()
{
    stopWatchdog();
    stopReadloop();
    spawnReadloop();
    bool success = simpleCommand(POWER_CONTROL_RESET_COMM, POWER_CONTROL_RESET_COMM_TIMEOUT);    
    spawnWatchdog();
    return success;
}

bool PowerController::shutdown(int shutdown_timeout, int restart_timeout)
{
    ControllerResponse response(POWER_CONTROL_SHUTDOWN, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(shutdown_timeout));
    response.addExpectedDataItem(ControllerData(restart_timeout));

    ControllerCommand  command(POWER_CONTROL_SHUTDOWN, response);
    command.addDataItem(shutdown_timeout);
    command.addDataItem(restart_timeout);

    return executeCommand(command);
}

bool PowerController::setSerialDebug(bool state)
{
    if(state == true)
        return setValue(POWER_CONTROL_SERIAL_DEBUG, HARDWARE_CONTROL_TIMEOUT, 1, serial_debug_mode);
    else
        return setValue(POWER_CONTROL_SERIAL_DEBUG, HARDWARE_CONTROL_TIMEOUT, 0, serial_debug_mode);
}

bool PowerController::resetADCMinMax(int output_number)
{
    return simpleCommand(POWER_CONTROL_RESET_MIN_MAX, HARDWARE_CONTROL_TIMEOUT, output_number);
}

bool PowerController::resetAllADCMinMax()
{
    return simpleCommand(POWER_CONTROL_RESET_ALL_MIN_MAX, HARDWARE_CONTROL_TIMEOUT);
}

bool PowerController::selectFullBattery()
{
    return getValue(POWER_CONTROL_SELECT_FULLEST_BAT, HARDWARE_CONTROL_TIMEOUT, selected_battery);
}

bool PowerController::selectBattery(int battery_number)
{
    return getValue(POWER_CONTROL_SELECT_FORCE_BAT, HARDWARE_CONTROL_TIMEOUT, battery_number, selected_battery);
}

bool PowerController::setDefaultOutputState(int output_number, bool state)
{
    if(output_number < 0 && output_number >= POWERCONTROLLER_NR_OUTPUTS)
    {
        ROS_WARN_NAMED(ROS_NAME, "Setting output %d, out of range.", output_number);
        return false;
    }

    ControllerResponse response(POWER_CONTROL_SET_DEFAULT_OUTPUT, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(output_number, "Invalid output id received at low-level power controller."));
    if(state == true)
        response.addExpectedDataItem(ControllerData(1, "Setting default output state unsuccessfull."));
    else
        response.addExpectedDataItem(ControllerData(0, "Setting default output state unsuccessfull."));

    ControllerCommand  command(POWER_CONTROL_SET_DEFAULT_OUTPUT, response);
    command.addDataItem(output_number);
    if(state == true)
        command.addDataItem(1);
    else
        command.addDataItem(0);
    
    return executeCommand(command);   
}

bool PowerController::getDefaultOutputState(int output_number)
{
    if(output_number < 0 && output_number >= POWERCONTROLLER_NR_OUTPUTS)
    {
        ROS_WARN_NAMED(ROS_NAME, "Getting output %d, out of range.", output_number);
        return false;
    }

    ControllerResponse response(POWER_CONTROL_GET_DEFAULT_OUTPUT, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(output_number, "Invalid output id received at low-level power controller."));
    response.addExpectedDataItem(ControllerData(set_default_outputs_[output_number]));
    ControllerCommand  command(POWER_CONTROL_GET_DEFAULT_OUTPUT, response);
    command.addDataItem(output_number);

    return executeCommand(command);
}
