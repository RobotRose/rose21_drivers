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

#include "rose21_lift_controller/lift_controller.hpp"

using namespace std;

LiftController::LiftController()
    : HardwareController<Serial>()
    , n_(ros::NodeHandle())
    , name_(ros::this_node::getName())
    , controller_enabled_(false)
    , requested_controller_enabled_(false)
    , sh_emergency_(SharedVariable<bool>("emergency"))
    , float_scale_(-1)
    , no_alarm_(false)
{
    loadParameters();

    comm_interface_         = Serial(name_, serial_port_, baud_rate_);

    // Publishers
    lift_pub_               = n_.advertise<rose_base_msgs::lift_state>("/lift_controller/lift/state", 1, true);
    joint_states_pub_       = n_.advertise<sensor_msgs::JointState>("/lift_controller/lift/joint_states", 1, true);
    bumpers_pub_            = n_.advertise<rose_base_msgs::bumpers_state>("/lift_controller/bumpers/state", 1, true);
    bumpers2_pub_           = n_.advertise<contact_sensor_msgs::bumpers>("/lift_controller/bumpers2/state", 1, true);

    // Subscribers
    lift_controller_enable_sub_  = n_.subscribe("/lift_controller/enabled", 1, &LiftController::CB_SetControllerState, this);
    lift_position_request_sub_   = n_.subscribe("/lift_controller/lift/command", 1, &LiftController::CB_LiftPositionRequest, this);

    sh_emergency_.host(false);
    sh_emergency_ = false;

    ROS_INFO("Started %s", name_.c_str());

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

    ROS_INFO("Stopped %s", name_.c_str());
}

void LiftController::loadParameters()
{
    // Get a private nodehandle to load the configurable parameters
    ros::NodeHandle pn = ros::NodeHandle("~");

    ROS_INFO("Loading '%s' parameters.", name_.c_str());

    ROS_ASSERT_MSG(pn.getParam("serial_port",       serial_port_), "Parameter serial_port must be specified.");
    ROS_ASSERT_MSG(pn.getParam("baud_rate",         baud_rate_), "Parameter baud_rate must be specified.");
    ROS_ASSERT_MSG(pn.getParam("major_version",     major_version_), "Parameter major_version must be specified.");     //! @todo OH [IMPR]: Move this such it is an configurable from the hardware controller.
    ROS_ASSERT_MSG(pn.getParam("minor_version",     minor_version_), "Parameter minor_version must be specified.");     //! @todo OH [IMPR]: Move this such it is an configurable from the hardware controller.

    ROS_ASSERT_MSG(pn.getParam("lift/min_pos",      lift_min_pos_), "Parameter lift/min_pos must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/max_pos",      lift_max_pos_), "Parameter lift/max_pos must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/min_speed",    lift_min_speed_), "Parameter lift/min_speed must be specified.");  // [0-255]
    ROS_ASSERT_MSG(pn.getParam("lift/max_speed",    lift_max_speed_), "Parameter lift/max_speed must be specified.");  // [0-255]
    
    ROS_ASSERT_MSG(pn.getParam("lift/p",            lift_p_), "Parameter lift/p must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/i",            lift_i_), "Parameter lift/i must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/i_lim",        lift_i_lim_), "Parameter lift/i_lim must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/p_scale",      lift_p_scale_), "Parameter lift/p_scale must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/i_scale",      lift_i_scale_), "Parameter lift/i_scale must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/hysteresis",   lift_hysteresis_), "Parameter lift/hysteresis must be specified.");

    ROS_ASSERT_MSG(pn.getParam("lift/base_joint",           base_joint_), "Parameter lift/base_joint must be specified.");
    ROS_ASSERT_MSG(pn.getParam("lift/arm_length",           lift_arm_length_), "Parameter lift/arm_length must be specified.");   
    ROS_ASSERT_MSG(pn.getParam("lift/motor_lift_distance",  motor_lift_distance_), "Parameter lift/motor_lift_distance must be specified.");   
    ROS_ASSERT_MSG(pn.getParam("lift/arm_lift_angle",       arm_lift_angle_), "Parameter lift/arm_lift_angle must be specified.");    

    // Load lift sensor calibration
    ROS_INFO("Loading lift sensor calibration table.");
    // Construct a map of strings
    XmlRpc::XmlRpcValue calibration_table;

    // Get the list of lists
    ROS_ASSERT_MSG(pn.getParam("lift/sensor_calibration", calibration_table), "Parameter lift/sensor_calibration must be specified.");
    ROS_ASSERT_MSG(calibration_table.getType() == XmlRpc::XmlRpcValue::TypeArray, "Parameter lift/sensor_calibration is not of type 'XmlRpc::XmlRpcValue::TypeArray'.");

    for (int i = 0; i < calibration_table.size(); ++i) 
    {
        XmlRpc::XmlRpcValue row = calibration_table[i];
        ROS_ASSERT_MSG(row.getType() == XmlRpc::XmlRpcValue::TypeArray, "Row is not a list.");
        ROS_ASSERT_MSG(row.size() == 2, "Incorrect calibration table structure, every row should have two entries ([mm] (double), [sensor reading](int)).");
        ROS_ASSERT_MSG(row[0].getType() == XmlRpc::XmlRpcValue::TypeDouble, "First value in row should be a double, every row should have two entries ([mm] (double), [sensor reading](int)).");
        ROS_ASSERT_MSG(row[1].getType() == XmlRpc::XmlRpcValue::TypeInt, "Second value in row should be an integer, every row should have two entries ([mm] (double), [sensor reading](int)).");

        double distance     = static_cast<double>(calibration_table[i][0]);
        double measurement  = (double)(static_cast<int>(calibration_table[i][1]));

        double factor = measurement/distance;
        ROS_INFO(" Length: %4.4fm | Sensor: %4.4f | Length/Sensor: %4.4f", distance, measurement, factor);

        sensor_calibration_data_.push_back(std::pair<double, double>(distance, measurement));
    }

    ROS_ASSERT_MSG(sensor_calibration_data_.size() > 1, "Sensor calibration table should contain at least two entries.");

    // sort sensor calibration data
    std::sort(sensor_calibration_data_.begin(), sensor_calibration_data_.end());

    ROS_INFO("Loaded '%s' parameters.", name_.c_str());
}

double LiftController::linearInterpolation(const double& y1, const double& y2, const double& x1, const double& x2, const double& x)
{
    double dx   = x2 - x1;
    double dy   = y2 - y1;
    double a    = dy/dx;
    double b    = (-a*x1) + y1;
    double interpolated = (a*x) + b; 
    return interpolated;
}

double LiftController::getMotorLength(const double& measurement)
{
    pair<double, double> first_pair;
    pair<double, double> second_pair;

    for(int i = 1; i < sensor_calibration_data_.size(); ++i)
    {
        first_pair     = sensor_calibration_data_[i - 1];
        second_pair    = sensor_calibration_data_[i];
        double dx = second_pair.second - first_pair.second;

        if(measurement <= second_pair.second)
            return linearInterpolation(first_pair.first, second_pair.first, first_pair.second, second_pair.second, measurement);
    }

    // Return last two pairs interpolation
    first_pair     = sensor_calibration_data_[sensor_calibration_data_.size() - 2];
    second_pair    = sensor_calibration_data_[sensor_calibration_data_.size() - 1];
    return linearInterpolation(first_pair.first, second_pair.first, first_pair.second, second_pair.second, measurement);
}

double LiftController::getSensorValue(const double& length)
{
    pair<double, double> first_pair;
    pair<double, double> second_pair;

    for(int i = 1; i < sensor_calibration_data_.size(); ++i)
    {
        pair<double, double> first_pair     = sensor_calibration_data_[i - 1];
        pair<double, double> second_pair    = sensor_calibration_data_[i];

        if(length <= second_pair.first)
            return linearInterpolation(first_pair.second, second_pair.second, first_pair.first, second_pair.first, length);
    }

    // Return last two pairs interpolation
    first_pair     = sensor_calibration_data_[sensor_calibration_data_.size() - 2];
    second_pair    = sensor_calibration_data_[sensor_calibration_data_.size() - 1];
    return linearInterpolation(first_pair.second, second_pair.second, first_pair.first, second_pair.first, length);
}



double LiftController::calculateLiftSetPoint(double lift_angle)
{
    // This was solved by wolfram alpha:  solve( D*sin(a) = sqrt( (L^2 - (b - cos(a)*D)^2)),L)
    // Giving:
    // L = -sqrt(-2 b D cos(a)+D^2 sin^2(a)+D^2 cos^2(a)+b^2)
    // L =  sqrt(-2 b D cos(a)+D^2 sin^2(a)+D^2 cos^2(a)+b^2)
    // b is the distance between the rotation point of the lift and the mounting point of the motor
    // D is the length of the arm that is attached to to the lift rotating point.

    double b = motor_lift_distance_;
    double D = lift_arm_length_;
    double angle = lift_angle - arm_lift_angle_;
    double L = sqrt(-2.0*b*D*cos(angle) + D*D*sin(angle)*sin(angle) + D*D*cos(angle)*cos(angle) + b*b);

    return getSensorValue(L);
}

sensor_msgs::JointState LiftController::calculateLiftJointAngle(int position)
{
    // This was solved by wolfram alpha: solve( D*sin(a) = sqrt( (L^2 - (b - cos(a)*D)^2)),a)
    // Giving:
    // a = -acos((b^2+D^2-L^2)/(2 b D))
    // a =  acos((b^2+D^2-L^2)/(2 b D))

    // a is the angle of the arm with the horizontal at the base_joint
    // c is the angle of the lift_top_link_
    // L is the length of the motor.
    // b is the distance between the rotation point of the lift and the mounting point of the motor
    // D is the length of the arm that is attached to to the lift rotating point.
    // arm_lift_angle_ will be the fixed angle between the arm and the lift

    double L = getMotorLength((double)position);
    double b = motor_lift_distance_;             //! @todo OH [IMPR]: extract from robot model?
    double D = lift_arm_length_;                 //! @todo OH [IMPR]: extract from robot model?

    // We take the negative solution because the x direction is in the direction of the front of the robot
    double a = -acos(((b*b) + (D*D) - (L*L)) / (2.0*b*D));
    // We add the fixed angle
    double a_lift = a + arm_lift_angle_;

    // Thus sensor value was: 
    double sv = calculateLiftSetPoint(a + arm_lift_angle_);

    ROS_DEBUG("Position: %d | L: %.4fm | b: %.4fm | D: %.4fm | a without fixed: %.4frad | a with fixed: %.4frad %.4fdeg, thus sv: %f", position, L, b, D, a, a_lift,((a_lift)* 180) / M_PI, sv);

    if(std::isnan(a_lift))
        ROS_ERROR("Calculated joint angle is NaN, this is probably caused by incorrect calibration data.");
    
    // Now publish the joint angle
    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();

    // Set base_joint
    joint_states.name.push_back(base_joint_);
    joint_states.position.push_back(a_lift);

    return joint_states;
}


void LiftController::publishLiftState()
{
    //! @todo OH [IMPR]: Only publish if changed.
    joint_states_pub_.publish(calculateLiftJointAngle(cur_position_));
}

void LiftController::publishBumpersState()
{
    rose_base_msgs::bumpers_state bumpers_state;
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

    ROS_DEBUG("Published bumpers status.");

    // Reset/start communication
    // resetComm();  
}

bool LiftController::enable()
{      
    // Check if enabled
    if(controller_enabled_)
        return true;   

    ROS_INFO("Enabling lift and bumper controller.");
    if(!get_comm_interface()->connect())   
        return false;

    // Reset/start communication
    resetComm();  

    if(!checkControllerID(LIFT_CONTROL_FIRMWARE_ID)) 
        return false;

    if(!checkFirmwareVersion(major_version_, minor_version_)) 
        return false;

    ROS_INFO("Lift and bumper controller controller with firmware version %d.%d connected.", received_firmware_major_version_, received_firmware_minor_version_);

    if( not getFloatScale() )
    {
        ROS_WARN("Could not get float scaling parameter for lift and bumper controller, controller will not be enabled.");
        return false;
    }

    if( not setParameters() )
    {
        ROS_WARN("Could not set default parameters for lift and bumper controller, controller will not be  not enabled.");
        return false;
    }

    int controller_enabled_integer = 0;
    setValue(LIFT_CONTROLLER_ENABLE, HARDWARE_CONTROL_TIMEOUT, 1, controller_enabled_integer);

    if (controller_enabled_integer == 1)
    {
        controller_enabled_ = true;      
        ROS_INFO("Lift and bumper controller enabled."); 
    }
    else
    {
        controller_enabled_ = false;
        ROS_WARN("Could not enable lift and bumper controller."); 
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
        ROS_INFO("Lift and bumper controller disabled."); 
    }
    else
    {
        controller_enabled_ = true;
        ROS_WARN("Could not disable lift and bumper controller."); 
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

bool LiftController::setParameters()
{
    bool all_success = true;
    all_success = setMinMaxLiftPosition(lift_min_pos_, lift_max_pos_) && all_success;
    all_success = setMinMaxLiftSpeed(lift_min_speed_, lift_max_speed_) && all_success;
    all_success = setControllerParameters(lift_p_, lift_i_, lift_i_lim_, lift_p_scale_, lift_i_scale_, lift_hysteresis_) && all_success;
    return all_success;
}

void LiftController::showState()
{
    ROS_INFO("Enabled                       : %d", controller_enabled_);
    ROS_INFO("Set lift speed                : %d", set_lift_speed_);
    ROS_INFO("Set lift speed                : %d%%", set_lift_speed_percentage_);
    ROS_INFO("Set lift position             : %d", set_lift_position_);
    ROS_INFO("Set lift position             : %d%%", set_lift_position_percentage_);

    ROS_INFO("Bumper states                 : %d|%d|%d|%d|%d|%d|%d|%d"  , bumper_states_[0]
                                                                                        , bumper_states_[1]
                                                                                        , bumper_states_[2]
                                                                                        , bumper_states_[3]
                                                                                        , bumper_states_[4]
                                                                                        , bumper_states_[5]
                                                                                        , bumper_states_[6]
                                                                                        , bumper_states_[7]);
    ROS_INFO("Lift Position                 : %d", cur_position_);
    ROS_INFO("Lift Position                 : %d%%", cur_position_percentage_);
    ROS_INFO("Lift Position Error           : %d", cur_position_error_);
    ROS_INFO("Lift P|I|PI|DUTY|DIRECTION    : %3.3f|%3.3f|%3.3f|%d|%d", lift_p_cmd_, lift_i_cmd_, lift_pi_cmd_, lift_duty_cycle_, lift_direction_);
    ROS_INFO("Is in position                : %d", is_in_position_);
    ROS_INFO("Is moving                     : %d", is_moving_);
    ROS_INFO("MIN/MAX position              : %d/%d", min_lift_position_, max_lift_position_);
    ROS_INFO("MIN/MAX speed                 : %d/%d", min_lift_speed_, max_lift_speed_);
    ROS_INFO("Safety state                  : %d|%d|%d|%d|%d|%d"    , safety_state_[0]
                                                                                    , safety_state_[1]
                                                                                    , safety_state_[2]
                                                                                    , safety_state_[3]
                                                                                    , safety_state_[4]
                                                                                    , safety_state_[5]);
    ROS_INFO("Safety Input State            : %d", safety_input_state_);
    ROS_INFO("Extra Input State             : %d", extra_input_state_);
    ROS_INFO("Extra Output State            : %d", extra_output_state_);

    for(int i = 0; i < 8; i++)
        ROS_INFO("ADC(%d) CUR/AVG/MIN/MAX   : %d, %d, %d, %d", i, adc_eng_[i][0], adc_eng_[i][1], adc_eng_[i][2], adc_eng_[i][3]);
}

bool LiftController::update()
{
    if(!controller_enabled_)
    {
        ROS_WARN("Trying to update lift while not enabled.");
        return false;
    }

    ROS_DEBUG("Updating lift and bumper controller state."); 
    
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
    all_success = getControllerStatus() && all_success;
    // all_success = getADCINT() && all_success; //! @todo OH: 

    handleSafety(); //! @todo OH: HACK should this be in update?

    return all_success;
}

//! @todo OH [IMPR]: Hack, what about all other safety states.
void LiftController::handleSafety()
{
    // Emergency input and button
    if(safety_input_state_ == 1 or safety_state_[0] == 1)
    {
        ROS_WARN_THROTTLE(0.1, "Emergency stop activated, wheels will be powered down.");
        sh_emergency_ = true;
    }

    // Process alarm state
    switch(safety_state_[5])
    {
        case 0:     // All OK
            break;
        case 1:     // Watchdog error
            no_alarm_ = false;
            ROS_WARN_THROTTLE(0.1, "The lift controller reported an watchdog error.");
            break;
        case 2:     // Force stop motor received
            ROS_WARN_THROTTLE(0.1, "The lift controller reported an forced motor stop.");
            no_alarm_ = false;
            break;
        default:
            ROS_ERROR_THROTTLE(0.1, "The lift controller reported an unknown error code.");
            no_alarm_ = false;
            break;
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
        ROS_ERROR("Cannot set pose while lift is disabled.");
        return false;
    }

    if(speed_precentage < 0 || speed_precentage > 100 || position_percentage < 0 || position_percentage > 100)
    {
        ROS_ERROR("Invalid percentages requested, position %d%%, speed %d%%.", position_percentage, speed_precentage);
        return false;
    } 

    ROS_INFO("Moving lift to %d%% at %d%% of maximal speed.", position_percentage, speed_precentage);       

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

bool LiftController::getFloatScale()
{
    float_scale_ = -1;
    return getValue(LIFT_CONTROLLER_GET_FLOAT_SCALE, HARDWARE_CONTROL_TIMEOUT, float_scale_);
}

bool LiftController::getControllerStatus()
{
    ControllerResponse response(LIFT_CONTROLLER_GET_CONTROLLER_STATUS, HARDWARE_CONTROL_TIMEOUT);

    response.addExpectedDataItem(ControllerData(lift_p_cmd_int_));
    response.addExpectedDataItem(ControllerData(lift_i_cmd_int_));
    response.addExpectedDataItem(ControllerData(lift_pi_cmd_int_));
    response.addExpectedDataItem(ControllerData(lift_duty_cycle_));
    response.addExpectedDataItem(ControllerData(lift_direction_));
    ControllerCommand  command(LIFT_CONTROLLER_GET_CONTROLLER_STATUS, response);

    if(executeCommand(command))
    {
        lift_p_cmd_     = lift_p_cmd_int_/float_scale_;
        lift_i_cmd_     = lift_i_cmd_int_/float_scale_;
        lift_pi_cmd_    = lift_pi_cmd_int_/float_scale_;
        return true;
    }
    else
    {
        lift_p_cmd_     = 0.0;
        lift_i_cmd_     = 0.0;
        lift_pi_cmd_    = 0.0;
        lift_duty_cycle_ = 0.0;
        lift_direction_  = 666; 
        return false;
    }
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

bool LiftController::setControllerParameters(double p, double i, int i_lim, int p_scale, int i_scale, int hysteresis)
{
    if(float_scale_ == -1)
    {
        ROS_WARN("setControllerParameters: Float scale was not set.");      
        return false;
    }

    ControllerResponse response(LIFT_CONTROLLER_SET_CONTROLLER_PARAMS, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(p*float_scale_, "Setting 'p' unsuccessful."));
    response.addExpectedDataItem(ControllerData(i*float_scale_, "Setting 'i' unsuccessful."));
    response.addExpectedDataItem(ControllerData(i_lim,          "Setting 'i_lim' unsuccessful."));
    response.addExpectedDataItem(ControllerData(p_scale,        "Setting 'p_scale' unsuccessful."));
    response.addExpectedDataItem(ControllerData(i_scale,        "Setting 'i_scale' unsuccessful."));
    response.addExpectedDataItem(ControllerData(hysteresis,     "Setting 'hysteresis' unsuccessful."));
    ControllerCommand  command(LIFT_CONTROLLER_SET_CONTROLLER_PARAMS, response);
    command.addDataItem(p*float_scale_);
    command.addDataItem(i*float_scale_);
    command.addDataItem(i_lim);
    command.addDataItem(p_scale);
    command.addDataItem(i_scale);
    command.addDataItem(hysteresis);

    return executeCommand(command);
}


bool LiftController::setMinMaxLiftPosition(int min_position, int max_position)
{
    ControllerResponse response(LIFT_CONTROLLER_SET_MINMAX_MOTOR_POS, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(min_position, min_lift_position_, "Setting minimal motor position unsuccessful."));
    response.addExpectedDataItem(ControllerData(max_position, max_lift_position_, "Setting maximal motor position unsuccessful."));
    ControllerCommand  command(LIFT_CONTROLLER_SET_MINMAX_MOTOR_POS, response);
    command.addDataItem(min_position);
    command.addDataItem(max_position);

    return executeCommand(command);
}

bool LiftController::setMinMaxLiftSpeed(int min_speed, int max_speed)
{
    ControllerResponse response(LIFT_CONTROLLER_SET_MINMAX_MOTOR_SPEED, HARDWARE_CONTROL_TIMEOUT);
    response.addExpectedDataItem(ControllerData(min_speed, min_lift_speed_, "Setting minimal motor speed unsuccessful."));
    response.addExpectedDataItem(ControllerData(max_speed, max_lift_speed_, "Setting maximal motor speed unsuccessful."));
    ControllerCommand  command(LIFT_CONTROLLER_SET_MINMAX_MOTOR_SPEED, response);
    command.addDataItem(min_speed);
    command.addDataItem(max_speed);

    return executeCommand(command);
}

void LiftController::CB_SetControllerState(const std_msgs::Bool::ConstPtr& enable_message)
{
    ROS_INFO("%s lift", enable_message->data?"Enabling":"Disabling");

    requested_controller_enabled_ = enable_message->data;
    if(enable_message->data) 
    {   
        if(controller_enabled_)
            ROS_INFO("Lift already enabled.");
        else if(enable())
            ROS_INFO("Lift successfully enabled.");
        else
            ROS_WARN("Lift NOT enabled!");
    }
    else
    {
        if(!controller_enabled_)
            ROS_INFO("Lift already disabled.");
        else if(disable())
            ROS_INFO("Lift successfully disabled");
        else
            ROS_WARN("Lift NOT disabled!");
    }          
}

void LiftController::CB_LiftPositionRequest(const rose_base_msgs::lift_command::ConstPtr& lift_command)
{
    if(no_alarm_)
        setPose(lift_command->speed_percentage, 100.0 - lift_command->position_percentage); 
    else
        ROS_WARN("Not setting requested lift position due to alarm state. (Alarm code: %d)", safety_state_[5]);
}
