/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2013/12/17
*       - File created.
*
* Description:
*   Controller that communicates using a Serial interface with the propeller that 
*   contols the wheels. It accepts wheel speeds and orientations and sends it to the
*   wheels.
* 
***********************************************************************************/

#ifndef PLATFORM_CONTROLLER_HPP
#define PLATFORM_CONTROLLER_HPP

#include <iostream>
#include <stdio.h>

#include <map>

#include "rose_hardware_controller/hardware_controller.hpp"

#include "rose_watchdogs/watchdog.hpp"

#include "rose_common/common.hpp"
 
#include "tf_helper/tf_helper.hpp"
 
#include "server_multiple_client/server_multiple_client.hpp"
#include "operator_messaging/operator_messaging.hpp"
#include "rose_shared_variables/shared_variable.hpp"
#include "action_result_message.hpp"

#include "rose_base_msgs/wheelunit_states.h"
#include "rose_base_msgs/wheelunit_statesAction.h"
#include "rose_base_msgs/wheelunit_statesActionGoal.h"
#include "rose_base_msgs/wheelunit_statesActionResult.h"
#include "rose_base_msgs/wheelunit_statesActionFeedback.h"

#include "odometry/odometry.hpp"
#include <map>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

#include "opteq_wheelunits_01/wheel_unit.hpp"

// Commands [string]
// Getters
#define PLATFORM_CONTROLLER_GET_VELOCITY              "200"
#define PLATFORM_CONTROLLER_GET_POSITION              "201"
#define PLATFORM_CONTROLLER_GET_CURRENT               "202"
#define PLATFORM_CONTROLLER_GET_MAX_CURRENT           "203"
#define PLATFORM_CONTROLLER_GET_POS_ERROR             "204"
#define PLATFORM_CONTROLLER_GET_VEL_ERROR             "205"
#define PLATFORM_CONTROLLER_GET_PIDOUT                "206"
#define PLATFORM_CONTROLLER_GET_POUT                  "207"
#define PLATFORM_CONTROLLER_GET_IOUT                  "208"
#define PLATFORM_CONTROLLER_GET_DOUT                  "209"
#define PLATFORM_CONTROLLER_GET_MAE_OUT               "210"
#define PLATFORM_CONTROLLER_GET_FE_OUT                "211"
#define PLATFORM_CONTROLLER_GET_ALARM_STATE           "212"
#define PLATFORM_CONTROLLER_GET_ALL_DRIVE_ENC_DIFF    "213"
#define PLATFORM_CONTROLLER_GET_COMPLETE_STATUS       "214"
#define PLATFORM_CONTROLLER_GET_DEBUG_STATUS          "215"

// Setters
#define PLATFORM_CONTROLLER_DRIVE_PID_VALS            "300"
#define PLATFORM_CONTROLLER_STEER_PID_VALS            "301"
#define PLATFORM_CONTROLLER_START_STOP_VALS           "302"
#define PLATFORM_CONTROLLER_ERROR_TIMERS              "303"

// Actions
#define PLATFORM_CONTROLLER_ENABLE                    "400"
#define PLATFORM_CONTROLLER_MOVE                      "401"
#define PLATFORM_CONTROLLER_BRAKE_MODE                "402"
#define PLATFORM_CONTROLLER_RESET_COMM                "403"           
#define PLATFORM_CONTROLLER_RESET_ALARM               "404"            
#define PLATFORM_CONTROLLER_RESET_PLATFORM            "405"
    
// PID values   
#define PLATFORM_CONTROLLER_DRIVE_KI               2000            // Division factor, but zero turns off!
#define PLATFORM_CONTROLLER_DRIVE_K                2000        //18000   // Division factor, but zero turns off!
#define PLATFORM_CONTROLLER_DRIVE_KD               -1000            // Division factor, but zero turns off!
#define PLATFORM_CONTROLLER_DRIVE_KP               0               // Velocity error to acceleration error factor
#define PLATFORM_CONTROLLER_DRIVE_ILIMIT           5000000
#define PLATFORM_CONTROLLER_DRIVE_SCALE            1000
#define PLATFORM_CONTROLLER_DRIVE_FEMAX            8
#define PLATFORM_CONTROLLER_DRIVE_MAX_CURR         5000
    
#define PLATFORM_CONTROLLER_STEER_KI               1000000         // Division factor, but zero turns off!
#define PLATFORM_CONTROLLER_STEER_K                400             // Division factor, but zero turns off!
#define PLATFORM_CONTROLLER_STEER_KD               0               // NOT IMPLEMENTED FOR STEER MOTORS
#define PLATFORM_CONTROLLER_STEER_KP               0               // NOT USED
#define PLATFORM_CONTROLLER_STEER_ILIMIT           10000000
#define PLATFORM_CONTROLLER_STEER_SCALE            100       
#define PLATFORM_CONTROLLER_STEER_FEMAX            6000
#define PLATFORM_CONTROLLER_STEER_MAX_CURR         6000

// Error timers in ms
#define PLATFORM_CONTROLLER_FOLLOWING_ERR_TIMER    2000
#define PLATFORM_CONTROLLER_CURRENT_ERR_TIMER      100
#define PLATFORM_CONTROLLER_CONNECTION_ERR_TIMER   400
#define PLATFORM_CONTROLLER_MAE_ERR_TIMER          75

// Generic platform defines
#define PLATFORM_CONTROLLER_CLK_FREQ               80000000.0      // [Hz] -> 80Mhz
#define PLATFORM_CONTROLLER_FIRMWARE_ID               1
#define PLATFORM_CONTROLLER_FIRMWARE_MAJOR_VERSION    2
#define PLATFORM_CONTROLLER_FIRMWARE_MINOR_VERSION    4

// platform_controller defines
#define MAX_RESET_TRIES                         5           // DISABLED, see reset function
#define RESET_INTERVAL                          2.0
#define VELOCITY_TIMEOUT                        0.5         // [s]

using namespace std;
using namespace rose_shared_variables;

class PlatformController : public HardwareController<Serial>
{
  protected:
    typedef ServerMultipleClient<rose_base_msgs::wheelunit_statesAction> SMC;

  public:
    PlatformController(string name, ros::NodeHandle n, string port_name, int baudrate);
    ~PlatformController();

    bool    update();
    bool    alarmStateOk();
    bool    enable();
    bool    disable();  
    bool    forceReset();   
    bool    reset(bool force_reset);    
    bool    isEnabled(string performing_action);
    bool    resetLowLevel();
    bool    resetLowLevelSafety();
    bool    resetLowLevelCommunication();
    bool    stopAllWheels();
    
    bool    setDrivePIDs(   int Ki, 
                            int K, 
                            int Kp, 
                            int Kd, 
                            int Ilimit, 
                            int PosScale, 
                            int VelScale,
                            int VelMax, 
                            int FeMax, 
                            int MaxCurr);

    bool    setSteerPIDs(   int Ki, 
                            int K, 
                            int Kp, 
                            int Kd, 
                            int Ilimit, 
                            int PosScale, 
                            int VelScale, 
                            int VelMax,
                            int FeMax, 
                            int MaxCurr);
    
    bool    setStartStopValues(int start_value_low_level, int stop_value_low_level);
    bool    setErrorTresholds(  const unsigned int& following_err_timer, 
                                const unsigned int& current_err_timer,
                                const unsigned int& connection_err_timer,
                                const unsigned int& mae_err_timer);
    bool    setActiveBrakeMode(bool active_brake);
    
    bool    getWheelUnitDrivePosition(WheelUnit& wheel_unit);
    bool    getAllWheelUnitDriveEncoderSpeeds();
    bool    getWheelUnitSteerVelocity(WheelUnit& wheel_unit);
    bool    getWheelUnitSteerPosition(WheelUnit& wheel_unit);
    bool    getWheelUnitVelocityError(WheelUnit& wheel_unit);
    bool    getWheelUnitPositionError(WheelUnit& wheel_unit);
    bool    getWheelUnitPIDOut(WheelUnit& wheel_unit); 
    bool    getWheelUnitPOut(WheelUnit& wheel_unit);
    bool    getWheelUnitIOut(WheelUnit& wheel_unit);
    bool    getWheelUnitDOut(WheelUnit& wheel_unit);
    bool    getWheelUnitMAE(WheelUnit& wheel_unit);
    bool    getWheelUnitFE(WheelUnit& wheel_unit);
    bool    getWheelUnitDriveCurrent(WheelUnit& wheel_unit);
    bool    getWheelUnitSteerCurrent(WheelUnit& wheel_unit);
    bool    getWheelUnitMaxDriveCurrent(WheelUnit& wheel_unit);
    bool    getWheelUnitMaxSteerCurrent(WheelUnit& wheel_unit);
    bool    getWheelUnitsDebugStates();
    bool    getAlarmStatus();
    bool    getStatus();
    
    vector<WheelUnit>& getWheelUnits();

    bool    sendWheelState();
    bool    writeWheelStates();
    void    publishWheelUnitStates();
    void    publishWheelUnitTransforms();
    void    CB_WheelUnitStatesRequest(const rose_base_msgs::wheelunit_statesGoalConstPtr& goal, SMC* smc);
    void    CB_cancelAllMovements();

    int     alarm_number_;
    int     alarm_byte_;

    int     reset_tries_;

  private:
    bool    stopWritten();
    bool    setAnglesZero();
    bool    setVelocitiesZero();

    ros::Publisher          wheelunit_states_pub_;
    vector<WheelUnit>       wheelunits_;
    vector<WheelUnit>       prev_wheelunits_;
    map<string, TFHelper>   wheelunit_transformers_;

    SMC*                    smc_;

    SharedVariable<bool>            sh_platform_controller_alarm_;
    SharedVariable<bool>            sh_platform_controller_reset_;
    SharedVariable<bool>            sh_emergency_;
    OperatorMessaging               operator_gui;
    rose::Watchdog                  velocity_watchdog_;

};

#endif // DRIVE_CONTROLLER_HPP
