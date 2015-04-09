/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2013/12/10
*       - File created.
*
* Description:
*   The lift contoller class
* 
***********************************************************************************/

#ifndef LIFT_CONTROLLER_HPP
#define LIFT_CONTROLLER_HPP

#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#include "rose_base_msgs/lift_command.h"
#include "rose_base_msgs/lift_state.h"
#include "rose_base_msgs/bumpers_state.h"
#include "std_msgs/Bool.h"
#include "contact_sensor_msgs/bumper.h"
#include "contact_sensor_msgs/bumpers.h"
#include "roscomm/conversion_bool.hpp"

#include "rose_shared_variables/shared_variable.hpp"

#include "rose_hardware_controller/hardware_controller.hpp"
#include "rose_hardware_comm/serial.hpp"


// Commands also see TXT specification file
// Read commands
#define LIFT_CONTROLLER_GET_BUMPERS                 "200" // (bumper states)
#define LIFT_CONTROLLER_GET_POS_PERCENTAGE          "201" // (lift position 0-100%)
#define LIFT_CONTROLLER_GET_SAFETY_INPUT            "202" // (safety input value)
#define LIFT_CONTROLLER_GET_EXTRA_INPUT             "203" // (extra input value)
#define LIFT_CONTROLLER_GET_SAFETY_STATE            "204" // (safety input, 3v3 OK, 5v OK, Vin OK, moving to button position)
#define LIFT_CONTROLLER_GET_ADC_RAW                 "205" // (actual, average, min, max)
#define LIFT_CONTROLLER_GET_ADC_VOL                 "206" // (actual, average, min, max)
#define LIFT_CONTROLLER_GET_ADC_ENG                 "207" // (average, min, max)
#define LIFT_CONTROLLER_GET_ADC_INT                 "208" // (integration time, integrated current mA)  
#define LIFT_CONTROLLER_GET_POS_ERROR               "209" // (position error 200-3400)
#define LIFT_CONTROLLER_GET_POS                     "210" // (position 200-3400)

#define LIFT_CONTROLLER_GET_IS_IN_POSITION          "212" // (bool)
#define LIFT_CONTROLLER_GET_IS_MOVING               "213" // (bool)
#define LIFT_CONTROLLER_GET_SERIAL_DEBUG_MODE       "214" // (bool)
#define LIFT_CONTROLLER_GET_SET_POS                 "215" // (position 200-3400)
#define LIFT_CONTROLLER_GET_SET_POS_PERCENTAGE      "216" // (0-100%)
#define LIFT_CONTROLLER_GET_SET_SPEED               "217" // (speed 0-255)
#define LIFT_CONTROLLER_GET_SET_SPEED_PERCENTAGE    "218" // (0-100%)

// Set commands
#define LIFT_CONTROLLER_SET_SAFETY_OUTPUT       "300" // bool                                -> bool (set value)
#define LIFT_CONTROLLER_SET_EXTRA_OUTPUT        "301" // bool                                -> bool (set value)
#define LIFT_CONTROLLER_SET_MINMAX_MOTOR_POS    "302" // int, int (200-3400, 200-3400)       -> int, int (set values)
#define LIFT_CONTROLLER_SET_MINMAX_MOTOR_SPEED  "303" // int, int (0-255, 0-255)             -> int, int (set values)

// Other commands
#define LIFT_CONTROLLER_ENABLE                  "400" // bool (enabled/disabled)                 -> bool (enabled/disabled)
#define LIFT_CONTROLLER_FORCE_STOP              "401" //                                         -> 
#define LIFT_CONTROLLER_SERIAL_DEBUG_MODE       "402" // bool (on/off)                           -> bool (on/off)
#define LIFT_CONTROLLER_MOVE_TO                 "403" // int, int (speed 0-100%, position 0-100%)-> int, int (received speed, received position) // non-blocking
#define LIFT_CONTROLLER_RESET_SPECIFIC_ADC      "404" // int (adc# 0-7)                          -> int (reset adc#0-7)
#define LIFT_CONTROLLER_RESET_ALL_ADC           "405" //                                         -> 
#define LIFT_CONTROLLER_RESET_ALARM_STATE       "406" //                                         -> 
#define LIFT_CONTROLLER_RESET_COMM              "407" //                                         -> 

// Platform properties
#define LIFT_CONTROLLER_CLK_FREQ                80000000.0      // [Hz] -> 80Mhz
#define LIFT_CONTROL_FIRMWARE_ID                2

using namespace std;
using namespace rose_shared_variables;

class LiftController : public HardwareController<Serial>
{
  public:
    // Functions
    LiftController();
    ~LiftController();

    void    publishLiftState();
    void    publishBumpersState();

    bool    enable();
    bool    disable();
    bool    isEnabled();
    void    resetState();
    bool    setDefaults();
    void    showState();
    bool    update();
    bool    getRequestedEnabledState();

    // Commands
    bool    setPose(int speed_precentage, int position_percentage);
    int     getPose();

    bool    forceMotorStop();
    bool    setSerialDebug(bool state);
    bool    resetADCMinMax(int output_number);
    bool    resetAllADCMinMax();
    bool    reset();
    bool    resetComm();
    bool    resetAlarm();

    bool    getBumperStates();
    bool    getSafetyInput();
    bool    getSafetyState();
    bool    getExtraInput();
    bool    getCurrentPosition();
    bool    getCurrentError();
    bool    getPositionPercentage();
    bool    getIsInPosition();
    bool    getIsMoving();
    bool    getSerialDebug();
    bool    getADCRAW();
    bool    getADCVOL();
    bool    getADCENG();
    bool    getADCINT();
    bool    getSetPosition();
    bool    getSetPositionPercentage();
    bool    getSetSpeed();
    bool    getSetSpeedPercentage();

    bool    setSafetyOutput(bool state);
    bool    setExtraOutput(bool state);

    bool    setMinMaxLiftPosition(int min_position, int max_position);
    bool    setMinMaxLiftSpeed(int min_speed, int max_speed);

    void    handleSafety();


    // Variables
    bool    controller_enabled_;
    bool    requested_controller_enabled_;

    // Controller state
    int     bumper_states_[8];

    int     adc_raw_[8][4];
    int     adc_vol_[8][4];
    int     adc_eng_[8][4];
    int     adc_int_[8][2];

    int     safety_input_state_;
    int     extra_input_state_;
    int     safety_output_state_;
    int     extra_output_state_;

    int     safety_state_[6];

    // Lift vars
    int     min_lift_position_;         
    int     max_lift_position_;          
    int     min_lift_speed_;          
    int     max_lift_speed_; 
    int     set_lift_speed_;           
    int     set_lift_speed_percentage_;           
    int     set_lift_position_;           
    int     set_lift_position_percentage_;           
    int     cur_position_;             
    int     cur_position_error_;       
    int     cur_position_percentage_;   
    int     is_in_position_;           
    int     is_moving_;                
    int     serial_debug_;        
    int     watchdog_treshold_;        


  protected:
    // Functions
    void                    loadParameters();

    // Callbacks
    void                    CB_SetControllerState(const std_msgs::Bool::ConstPtr& enable);
    void                    CB_LiftPositionRequest(const rose_base_msgs::lift_command::ConstPtr& lift_command);

    // Variables
    ros::NodeHandle         n_;
    std::string             name_;

    ros::Publisher          lift_pub_;
    ros::Publisher          bumpers_pub_;
    ros::Publisher          bumpers2_pub_;
    ros::Subscriber         lift_controller_enable_sub_;
    ros::Subscriber         lift_position_request_sub_;

    std::string serial_port_;
    int         baud_rate_;
    int         major_version_;
    int         minor_version_;

    int lift_min_pos_;
    int lift_max_pos_;
    int lift_min_speed_;
    int lift_max_speed_;

    std::map<int, std::vector<rose_geometry::Point>> bumper_footprints_;

    SharedVariable<bool>    sh_emergency_;

    
};

#endif // LIFT_CONTROLLER_HPP
