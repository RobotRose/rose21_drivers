/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/08/22
*       - File created.
*
* Description:
*   ROS driver class for the power controller board
* 
***********************************************************************************/

#ifndef POWER_CONTROLLER_HPP
#define POWER_CONTROLLER_HPP

#include <iostream>
#include <stdio.h>

#include "rose_hardware_controller/hardware_controller.hpp"
#include "rose21_platform/battery_state.h"
#include "std_msgs/Int32.h"

#include "rose_common/common.hpp"
 
#include "shared_variables/shared_variable.hpp"

#include "operator_messaging/operator_messaging.hpp"


// Getters
#define POWER_CONTROL_GET_POWER_OUTPUTS     "200"
#define POWER_CONTROL_GET_BATTERY           "201"
#define POWER_CONTROL_GET_SWITCH_VIN        "202"
#define POWER_CONTROL_GET_WARNING_VIN       "203"
#define POWER_CONTROL_GET_MINIMAL_VIN       "204"
#define POWER_CONTROL_GET_SAFETY_IN         "205"
#define POWER_CONTROL_GET_EXTRA_IN          "206"
#define POWER_CONTROL_GET_ADC_RAW           "207"
#define POWER_CONTROL_GET_ADC_VOL           "208"
#define POWER_CONTROL_GET_ADC_ENG           "209"
#define POWER_CONTROL_GET_ADC_INT           "210"       // TODO OH: Not implemented yet
#define POWER_CONTROL_GET_BAT_VOLT_RAW      "211"   
#define POWER_CONTROL_GET_BAT_VOLT_AVG      "212"   
#define POWER_CONTROL_GET_SWITCH_SOUND      "213"   
#define POWER_CONTROL_GET_ALARM_SOUND       "214"   
#define POWER_CONTROL_GET_ALARM_SOUND_INTER "215"   

#define POWER_CONTROL_GET_SHUTDOWN_TIME     "217"   
#define POWER_CONTROL_GET_RESTART_TIME      "218"   
#define POWER_CONTROL_GET_SET_TIMER_VALUE   "219"   
#define POWER_CONTROL_GET_CUR_TIMER_VALUE   "220"  

// Setters
#define POWER_CONTROL_SET_AUTO_SWITCH_VIN   "300"
#define POWER_CONTROL_SET_WARNING_VIN       "301"
#define POWER_CONTROL_SET_MINIMAL_VIN       "302"
#define POWER_CONTROL_SET_EXTRA_OUT         "303"
#define POWER_CONTROL_SET_AUTO_SWITCH_MODE  "304"
#define POWER_CONTROL_SET_POWER_OUTPUT      "305"
#define POWER_CONTROL_SET_SWITCH_SOUND      "306"
#define POWER_CONTROL_SET_ALARM_SOUND       "307"
#define POWER_CONTROL_SET_ALARM_INTERVAL    "308"

// Others
#define POWER_CONTROL_RESET_COMM            "400"
#define POWER_CONTROL_SHUTDOWN              "401"
#define POWER_CONTROL_SERIAL_DEBUG          "402"
#define POWER_CONTROL_RESET_MIN_MAX         "403"
#define POWER_CONTROL_RESET_ALL_MIN_MAX     "404"
#define POWER_CONTROL_SELECT_FULLEST_BAT    "405"
#define POWER_CONTROL_SELECT_FORCE_BAT      "406"

#define POWER_CONTROL_SET_DEFAULT_OUTPUT    "408"
#define POWER_CONTROL_GET_DEFAULT_OUTPUT    "409"

// Responses Timeouts [sec]
#define POWER_CONTROL_DEFAULT_TIMEOUT           1
#define POWER_CONTROL_RESET_COMM_TIMEOUT        5

// Generic platform defines
#define POWER_CONTROL_CLK_FREQ                  80000000.0      // [Hz] -> 80Mhz
#define POWER_CONTROL_FIRMWARE_ID               3
#define POWER_CONTROL_FIRMWARE_MAJOR_VERSION    1
#define POWER_CONTROL_FIRMWARE_MINOR_VERSION    3
#define SHUTDOWN_TIMER_ID                       1      
#define RESTART_TIMER_ID                        2         


// Default parameters
#define LOW_BATTERY_SOUND_ENABLED               true
#define LOW_BATTERY_BEEP_INTERVAL               10000       // [ms]
#define SWITCH_SOUND_ENABLED                    true
#define MINIMAL_BATTERY_VOLTAGE                 22400       // [mV]
#define WARNING_BATTERY_VOLTAGE                 23000       // [mV]
#define POWER_CONTROLLER_BATTERY_FULL_VOLTAGE   27400       // [mV]
#define AUTO_SWITCH_MODE_ENABLED                true
#define AUTO_SWITCH_BATTERY_VOLTAGE             23100       // [mV]
#define DEFAULT_OUTPUT_STATES                   {true, true, true, true, true, true}       // TRUE = default on, FALSE = default off
#define POWERCONTROLLER_NR_OUTPUTS              6
#define GUI_BAT_WARNING_PRECENTAGE              10.0        // [%]
#define GUI_BAT_WARNING_INTERVAL                10.0        // [s]

using namespace shared_variables;

class PowerController : public HardwareController<Serial>
{
  public:
    PowerController(string name, ros::NodeHandle n, string port_name, int baudrate);
    ~PowerController();

    bool enable();
    bool setDefaults();

    void resetState();

    bool update();
    bool completeUpdate();

    rose_base_msgs::bumpers_state getBatteryState(int battery_number);
    rose_base_msgs::bumpers_state getCombinedBatteryState();
    void publishBatteryStates();

    void showState();

    void togglePowerOutput(int index);

    bool increaseSoundInterval();
    bool decreaseSoundInterval();
    
    // Getters
    bool getPowerOutputs();
    bool getSelectedBattery();
    bool getSwitchVoltage();
    bool getWarningVoltage();
    bool getMinimalVoltage();
    bool getSafetyInput();
    bool getExtraInput();
    bool getADCRAW();
    bool getADCVOL();
    bool getADCENG();
    bool getADCINT();
    bool getBatteryVoltageRaw();
    bool getBatteryVoltageAverage();
    bool getSwitchSoundState();
    bool getAlarmSoundState();
    bool getAlarmSoundInterval();
    bool getShutdownTime();
    bool getRestartTime();
    bool getSetTimerValue(int timer_id, int& timer_set_value);
    bool getCurrentTimerValue(int timer_id, int& timer_current_value);

    bool setAutoSwitchVoltage(int mv);
    bool setWarningVoltage(int mv);
    bool setMinimalVoltage(int mv);
    bool setExtraOutput(bool state);
    bool setAutoSwitchMode(bool state);
    bool setPowerOutput(int index, int state);
    bool getPowerOutput(int index);
    bool setSwitchSound(bool state);
    bool setAlarmSound(bool state);
    bool setAlarmSoundInterval(int interval);

    bool resetComm();
    bool shutdown(int shutdown_timeout, int restart_timeout);
    bool setSerialDebug(bool state);
    bool resetADCMinMax(int output_number);
    bool resetAllADCMinMax();
    bool selectFullBattery();
    bool selectBattery(int battery_number);
    bool setDefaultOutputState(int output_number, bool state);
    bool getDefaultOutputState(int output_number);


  private:
    ros::Publisher          selected_battery_pub_;
    ros::Publisher          battery1_state_pub_;
    ros::Publisher          battery2_state_pub_;
    ros::Publisher          battery_combined_state_pub_;

    SharedVariable<std::vector<int>>    sh_power_output_states_;
    SharedVariable<bool>                sh_emergency_;
    OperatorMessaging                   operator_gui_;
    ros::Time                           bat_warn_time_;

  public:               //! @todo OH: Not public?
    //! @todo OH: add underscores
    // State variables
    int     set_default_outputs_[6];
    int     selected_battery;
    int     switch_voltage;
    int     warning_voltage;
    int     minimal_voltage;
    int     safety_input_state;
    int     extra_input_state;
    int     extra_output_state;
    int     adc_raw[16][4];
    int     adc_vol[16][4];
    int     adc_eng[16][4];
    int     adc_int[16][2];

    int     battery_auto_mode;
    int     battery_switch_sound;
    int     battery_voltage_raw[2];
    int     battery_voltage_avg[2];
    int     alarm_sound;
    int     alarm_sound_interval;
    int     watchdog_treshold;
    int     serial_debug_mode;

    int     shutdown_timer_;
    int     restart_timer_;

};

#endif // POWER_CONTROLLER_HPP
