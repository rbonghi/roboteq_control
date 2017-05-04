
#include "configurator/motor_param.h"


#define PARAM_ENCODER_STRING "/encoder"

MotorParamConfigurator::MotorParamConfigurator(const ros::NodeHandle& nh, roboteq::serial_controller *serial, std::string name, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + name;
    // Roboteq motor number
    mNumber = number + 1;
    // Set false on first run
    setup_param = false;
    setup_encoder = false;
}

void MotorParamConfigurator::initConfigurator(bool load_from_board)
{
    double ratio;
    // Check if exist ratio variable
    if(nh_.hasParam(mName + "/ratio"))
    {
        double temp_double;
        nh_.getParam(mName + "/ratio", temp_double);
        // Set Ratio
        ratio = temp_double;
    } else
    {
        nh_.setParam(mName + "/ratio", 1.0);
        // Set Ratio
        ratio = 1.0;
    }

    // Check if is required load paramers
    if(load_from_board)
    {
        // Load parameters from roboteq
        getParamFromRoboteq();
        // Load encoder properties from roboteq
        getEncoderFromRoboteq();
    }

    // Initialize parameter dynamic reconfigure
    ds_param = new dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig>::CallbackType cb_param = boost::bind(&MotorParamConfigurator::reconfigureCBParam, this, _1, _2);
    ds_param->setCallback(cb_param);

    // Initialize encoder dynamic reconfigure
    ds_encoder = new dynamic_reconfigure::Server<roboteq_control::RoboteqEncoderConfig>(ros::NodeHandle(mName + PARAM_ENCODER_STRING));
    dynamic_reconfigure::Server<roboteq_control::RoboteqEncoderConfig>::CallbackType cb_encoder = boost::bind(&MotorParamConfigurator::reconfigureCBEncoder, this, _1, _2);
    ds_encoder->setCallback(cb_encoder);

    // Initialize pid type dynamic reconfigure
    ds_pid_type = new dynamic_reconfigure::Server<roboteq_control::RoboteqPIDtypeConfig>(ros::NodeHandle(mName + "/pid"));
    dynamic_reconfigure::Server<roboteq_control::RoboteqPIDtypeConfig>::CallbackType cb_pid_type = boost::bind(&MotorParamConfigurator::reconfigureCBPIDtype, this, _1, _2);
    ds_pid_type->setCallback(cb_pid_type);

    // Get PPR Encoder parameter
    double ppr;
    nh_.getParam(mName + PARAM_ENCODER_STRING + "/PPR", ppr);
    _reduction = ppr;
    // Check if exist ratio variable
    if(nh_.hasParam(mName + PARAM_ENCODER_STRING + "/position"))
    {
        int position;
        nh_.getParam(mName + PARAM_ENCODER_STRING + "/position", position);
        // Read position if before (1) multiply with ratio
        if(position) {
            _reduction *= ratio;
        }
    }
    // Multiply for quadrature
    // TODO check for encoder with single channel
    _reduction *= 4;

    //ROS_INFO_STREAM("reduction:" << _reduction);
}

void MotorParamConfigurator::setOperativeMode(int type)
{
    // Update operative mode
    mSerial->setParam("MMOD", std::to_string(mNumber) + " " + std::to_string(type));
}

int MotorParamConfigurator::getOperativeMode()
{
    // Operative mode reference in [pag 321]
    string str_mode = mSerial->getParam("MMOD", std::to_string(mNumber));
    // Get sign from roboteq board
    int mode = boost::lexical_cast<int>(str_mode);
    // Set parameter
    nh_.setParam(mName + "/operating_mode", mode);

    return mode;
}

void MotorParamConfigurator::reconfigureCBPIDtype(roboteq_control::RoboteqPIDtypeConfig &config, uint32_t level)
{
    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_pid_type)
    {
      _last_pid_type_config = config;
      default_pid_type_config = _last_pid_type_config;
      setup_pid_type = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload config with default
        config = default_pid_type_config;
    }
}

void MotorParamConfigurator::getParamFromRoboteq()
{
    try
    {
        // Load Ratio
        double ratio;
        nh_.getParam(mName + "/ratio", ratio);

        // Motor direction {1 (Clockwise), -1 (Underclockwise)}
        string str_mdir = mSerial->getParam("MDIR", std::to_string(mNumber));
        // Get sign from roboteq board
        int sign = boost::lexical_cast<int>(str_mdir) ? -1 : 1;
        // Set parameter
        nh_.setParam(mName + "/rotation", sign);

        // Stall detection [pag. 310]
        string str_stall = mSerial->getParam("BLSTD", std::to_string(mNumber));
        int stall = boost::lexical_cast<int>(str_stall);
        // Set params
        nh_.setParam(mName + "/stall_detection", stall);

        // Get Max Amper limit = alim / 10 [pag 306]
        string str_alim = mSerial->getParam("ALIM", std::to_string(mNumber));
        unsigned int tmp = boost::lexical_cast<unsigned int>(str_alim);
        double alim = ((double) tmp) / 10.0;
        // Set params
        nh_.setParam(mName + "/amper_limit", alim);

        // Max power forward [pag. 323]
        string str_max_fw = mSerial->getParam("MXPF", std::to_string(mNumber));
        // Get max forward
        int max_forward = boost::lexical_cast<unsigned int>(str_max_fw);
        // Set parameter
        nh_.setParam(mName + "/max_acceleration", max_forward);

        // Max power forward reverse [pag. 324]
        string str_max_re = mSerial->getParam("MXPR", std::to_string(mNumber));
        // Get max reverse
        int max_reverse = boost::lexical_cast<unsigned int>(str_max_re);
        // Set parameter
        nh_.setParam(mName + "/max_deceleration", max_reverse);

        // Get Max RPM motor
        string str_rpm_motor = mSerial->getParam("MXRPM", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_motor = boost::lexical_cast<unsigned int>(str_rpm_motor);
        // Convert in max RPM
        double max_rpm = ((double) rpm_motor) / ratio;
        // Set parameter
        nh_.setParam(mName + "/max_speed", max_rpm);

        // Get Max RPM acceleration rate
        string str_rpm_acceleration_motor = mSerial->getParam("MAC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_acceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_acceleration_motor);
        // Convert in max RPM
        double rpm_acceleration = ((double) rpm_acceleration_motor) / ratio;
        // Set parameter
        nh_.setParam(mName + "/max_acceleration", rpm_acceleration);

        // Get Max RPM deceleration rate
        string str_rpm_deceleration_motor = mSerial->getParam("MDEC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_deceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_deceleration_motor);
        // Convert in max RPM
        double rpm_deceleration = ((double) rpm_deceleration_motor) / ratio;
        // Set parameter
        nh_.setParam(mName + "/max_deceleration", rpm_deceleration);

    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void MotorParamConfigurator::getEncoderFromRoboteq() {
    try
    {
        // Get Encoder Usage - reference pag. 315
        string str_emode = mSerial->getParam("EMOD", std::to_string(mNumber));
        // Get PPR from roboteq board
        int emod = boost::lexical_cast<unsigned int>(str_emode);
        // 3 modes:
        // 0 - Unsed
        // 1 - Command mode
        // 2 - Feedback
        int command = (emod & 0b11);
        int type = 0;
        if(command == 2)
        {
            switch(emod - command)
            {
            // One channel
            case 16:
                type = 1;
                break;
            // Two channel mode
            case 48:
                type = 2;
                break;
            // Other configurations
            default:
                type = 0;
                break;
            }
        } else
        {
            // Set unused
            type = 0;
        }
        // ROS_INFO_STREAM("command:" << command << " type=" << type);
        // Set parameter
        nh_.setParam(mName + PARAM_ENCODER_STRING + "/channels", type);

        // Get Encoder PPR (Pulse/rev) [pag. 316]
        string str_ppr = mSerial->getParam("EPPR", std::to_string(mNumber));
        // Get PPR from roboteq board
        int ppr = boost::lexical_cast<unsigned int>(str_ppr);
        // Set parameter
        nh_.setParam(mName + PARAM_ENCODER_STRING + "/PPR", ppr);

        // Get Encoder ELL - Min limit [pag. 314]
        string str_ell = mSerial->getParam("ELL", std::to_string(mNumber));
        // Get PPR from roboteq board
        int ell = boost::lexical_cast<unsigned int>(str_ell);
        // Set parameter
        nh_.setParam(mName + PARAM_ENCODER_STRING + "/encoder_low_count_limit", ell);

        // Get Encoder EHL - Max limit [pag. 311]
        string str_ehl = mSerial->getParam("EHL", std::to_string(mNumber));
        // Get PPR from roboteq board
        int ehl = boost::lexical_cast<unsigned int>(str_ehl);
        // Set parameter
        nh_.setParam(mName + PARAM_ENCODER_STRING + "/encoder_high_count_limit", ehl);

        // Get Encoder EHOME - Home count [pag. 313]
        string str_home = mSerial->getParam("EHOME", std::to_string(mNumber));
        // Get PPR from roboteq board
        int home = boost::lexical_cast<unsigned int>(str_home);
        // Set parameter
        nh_.setParam(mName + PARAM_ENCODER_STRING + "/encoder_home_count", home);


    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void MotorParamConfigurator::reconfigureCBParam(roboteq_control::RoboteqParameterConfig &config, uint32_t level) {

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_param)
    {
      _last_param_config = config;
      default_param_config = _last_param_config;
      setup_param = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload config with default
        config = default_param_config;
    }

    if(config.load_roboteq)
    {
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
        // Launch param load
        getParamFromRoboteq();
        // Skip other read
        return;
    }

    // Update ratio
    // Get old max speed, acceleration and deceleartion and evaluate new equivalent value
    if(_last_param_config.ratio != config.ratio)
    {
        // Get Max RPM motor
        string str_rpm_motor = mSerial->getParam("MXRPM", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_motor = boost::lexical_cast<unsigned int>(str_rpm_motor);
        // Update with new max speed
        config.max_speed = ((double) rpm_motor) / config.ratio;

        // Get Max RPM acceleration rate
        string str_rpm_acceleration_motor = mSerial->getParam("MAC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_acceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_acceleration_motor);
        // Convert in max RPM
        config.max_acceleration = ((double) rpm_acceleration_motor) / config.ratio;

        // Get Max RPM deceleration rate
        string str_rpm_deceleration_motor = mSerial->getParam("MDEC", std::to_string(mNumber));
        // Get RPM from board
        unsigned int rpm_deceleration_motor = boost::lexical_cast<unsigned int>(str_rpm_deceleration_motor);
        // Convert in max RPM
        config.max_deceleration = ((double) rpm_deceleration_motor) / config.ratio;
    }

    if(_last_param_config.rotation != config.rotation)
    {
        // Update direction
        int direction = (config.rotation == -1) ? 1 : 0;
        mSerial->setParam("MDIR", std::to_string(mNumber) + " " + std::to_string(direction));
    }
    // Stall detection [pag. 310]
    if(_last_param_config.stall_detection != config.stall_detection)
    {
        // Update stall detection value
        mSerial->setParam("BLSTD", std::to_string(mNumber) + " " + std::to_string(config.stall_detection));
    }
    // Get Max Amper limit = alim * 10 [pag 306]
    if(_last_param_config.amper_limit != config.amper_limit)
    {
        // Update stall detection value
        int alim = config.amper_limit * 10;
        mSerial->setParam("ALIM", std::to_string(mNumber) + " " + std::to_string(alim));
    }
    // Max power forward [pag. 323]
    if(_last_param_config.max_forward != config.max_forward)
    {
        // Update max forward
        mSerial->setParam("MXPF", std::to_string(mNumber) + " " + std::to_string(config.max_forward));
    }
    // Max power forward reverse [pag. 324]
    if(_last_param_config.max_forward != config.max_reverse)
    {
        // Update max forward reverse
        mSerial->setParam("MXPR", std::to_string(mNumber) + " " + std::to_string(config.max_reverse));
    }

    // Set Max RPM motor
    if(_last_param_config.max_speed != config.max_speed)
    {
        // Update max RPM motor
        long int max_speed_motor = config.ratio * config.max_speed;
        mSerial->setParam("MXRPM", std::to_string(mNumber) + " " + std::to_string(max_speed_motor));
    }

    // Set Max RPM acceleration rate
    if(_last_param_config.max_acceleration != config.max_acceleration)
    {
        // Update max acceleration RPM/s motor
        long int max_acceleration_motor = config.ratio * config.max_acceleration;
        mSerial->setParam("MAC", std::to_string(mNumber) + " " + std::to_string(max_acceleration_motor));
    }
    // Set Max RPM deceleration rate
    if(_last_param_config.max_deceleration != config.max_deceleration)
    {
        // Update max deceleration RPM/s motor
        long int max_deceleration_motor = config.ratio * config.max_deceleration;
        mSerial->setParam("MDEC", std::to_string(mNumber) + " " + std::to_string(max_deceleration_motor));
    }

    // Update last configuration
    _last_param_config = config;
}

void MotorParamConfigurator::reconfigureCBEncoder(roboteq_control::RoboteqEncoderConfig &config, uint32_t level) {

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_encoder)
    {
      _last_encoder_config = config;
      default_encoder_config = _last_encoder_config;
      setup_encoder = true;
      return;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
        // Overload default configuration
        config = default_encoder_config;
    }

    if(config.load_roboteq)
    {
        ROS_INFO_STREAM("LOAD from Roboteq");
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
        // Launch encoder load
        getEncoderFromRoboteq();
        // Skip other read
        return;
    }

    // Set Encoder Usage - reference pag. 315
    if(_last_encoder_config.channels != config.channels)
    {
        int channels;
        switch(config.channels)
        {
        case 1:
            // set in feedback mode with one channel
            channels = 2 + 16;
            break;
        case 2:
            // set in feedback mode with one channel
            channels = 2 + 48;
            break;
        default:
            channels = 0;
            break;
        }
        // Update operative mode
        mSerial->setParam("EMOD", std::to_string(mNumber) + " " + std::to_string(channels));
    }
    // Set Encoder PPR
    if(_last_encoder_config.PPR != config.PPR)
    {
        // Update operative mode
        mSerial->setParam("EPPR", std::to_string(mNumber) + " " + std::to_string(config.PPR));
    }
    // Set Encoder ELL - Min limit [pag. 314]
    if(_last_encoder_config.encoder_low_count_limit != config.encoder_low_count_limit)
    {
        // Update operative mode
        mSerial->setParam("ELL", std::to_string(mNumber) + " " + std::to_string(config.encoder_low_count_limit));
    }
    // Set Encoder EHL - Max limit [pag. 311]
    if(_last_encoder_config.encoder_high_count_limit != config.encoder_high_count_limit)
    {
        // Update operative mode
        mSerial->setParam("EHL", std::to_string(mNumber) + " " + std::to_string(config.encoder_high_count_limit));
    }
    // Set Encoder EHOME - Home count [pag. 313]
    if(_last_encoder_config.encoder_home_count != config.encoder_home_count)
    {
        // Update operative mode
        mSerial->setParam("EHOME", std::to_string(mNumber) + " " + std::to_string(config.encoder_home_count));
    }

    // Update last configuration
    _last_encoder_config = config;

}
