
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
    setup_ = false;
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

    // Initialize parameter dynamic reconfigure
    ds_param = new dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig>::CallbackType cb_param = boost::bind(&MotorParamConfigurator::reconfigureCBParam, this, _1, _2);
    ds_param->setCallback(cb_param);

    // Initialize encoder dynamic reconfigure
    ds_encoder = new dynamic_reconfigure::Server<roboteq_control::RoboteqEncoderConfig>(ros::NodeHandle(mName + PARAM_ENCODER_STRING));
    dynamic_reconfigure::Server<roboteq_control::RoboteqEncoderConfig>::CallbackType cb_encoder = boost::bind(&MotorParamConfigurator::reconfigureCBEncoder, this, _1, _2);
    ds_encoder->setCallback(cb_encoder);
}

void MotorParamConfigurator::getParamFromRoboteq() {
    try
    {
        // Load Ratio
        double ratio;
        nh_.getParam(mName + "/ratio", ratio);

        // Motor direction {1 (Clockwise), -1 (Underclockwise)}
        if(mSerial->query("MDIR", std::to_string(mNumber), "~"))
        {
            // Get sign from roboteq board
            int sign = boost::lexical_cast<int>(mSerial->get()) ? -1 : 1;
            // Set parameter
            nh_.setParam(mName + "/rotation", sign);
        }
        // Stall detection [pag. 310]
        if(mSerial->query("BLSTD", std::to_string(mNumber), "~"))
        {
            int stall = boost::lexical_cast<int>(mSerial->get());
            // Set params
            nh_.setParam(mName + "/stall_detection", stall);
        }

        // Get Max Amper limit = alim / 10 [pag 306]
        if(mSerial->query("ALIM", std::to_string(mNumber), "~"))
        {
            unsigned int tmp = boost::lexical_cast<unsigned int>(mSerial->get());
            double alim = ((double) tmp) / 10.0;
            // Set params
            nh_.setParam(mName + "/amper_limit", alim);
        }

        // Max power forward [pag. 323]
        if(mSerial->query("MXPF", std::to_string(mNumber), "~"))
        {
            // Get max forward
            int max_forward = boost::lexical_cast<unsigned int>(mSerial->get());
            // Set parameter
            nh_.setParam(mName + "/max_acceleration", max_forward);

        }
        // Max power forward reverse [pag. 324]
        if(mSerial->query("MXPR", std::to_string(mNumber), "~"))
        {
            // Get max reverse
            int max_reverse = boost::lexical_cast<unsigned int>(mSerial->get());
            // Set parameter
            nh_.setParam(mName + "/max_deceleration", max_reverse);
        }

        // Get Max RPM motor
        if(mSerial->query("MXRPM", std::to_string(mNumber), "~"))
        {
            // Get RPM from board
            unsigned int rpm_motor = boost::lexical_cast<unsigned int>(mSerial->get());
            // Convert in max RPM
            double max_rpm = ((double) rpm_motor) / ratio;
            // Set parameter
            nh_.setParam(mName + "/max_speed", max_rpm);
        }
        // Get Max RPM acceleration rate
        if(mSerial->query("MAC", std::to_string(mNumber), "~"))
        {
            // Get RPM from board
            unsigned int rpm_acceleration_motor = boost::lexical_cast<unsigned int>(mSerial->get());
            // Convert in max RPM
            double rpm_acceleration = ((double) rpm_acceleration_motor) / ratio;
            // Set parameter
            nh_.setParam(mName + "/max_acceleration", rpm_acceleration);
        }
        // Get Max RPM deceleration rate
        if(mSerial->query("MDEC", std::to_string(mNumber), "~"))
        {
            // Get RPM from board
            unsigned int rpm_deceleration_motor = boost::lexical_cast<unsigned int>(mSerial->get());
            // Convert in max RPM
            double rpm_deceleration = ((double) rpm_deceleration_motor) / ratio;
            // Set parameter
            nh_.setParam(mName + "/max_deceleration", rpm_deceleration);
        }

    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void MotorParamConfigurator::getEncoderFromRoboteq() {
    try
    {
        // Get Encoder PPR - Will be move in Encoder page
        if(mSerial->query("EPPR", std::to_string(mNumber), "~"))
        {
            // Get PPR from roboteq board
            int ppr = boost::lexical_cast<unsigned int>(mSerial->get());
            // Set parameter
            nh_.setParam(mName + PARAM_ENCODER_STRING + "/PPR", ppr);
        }


    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void MotorParamConfigurator::reconfigureCBParam(roboteq_control::RoboteqParameterConfig &config, uint32_t level) {

    if(config.load_roboteq)
    {
        ROS_INFO_STREAM("LOAD from Roboteq");
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
    }

}

void MotorParamConfigurator::reconfigureCBEncoder(roboteq_control::RoboteqEncoderConfig &config, uint32_t level) {

    if(config.load_roboteq)
    {
        ROS_INFO_STREAM("LOAD from Roboteq");
        //if someone sets again the request on the parameter server, prevent looping
        config.load_roboteq = false;
    }

    if(config.restore_defaults)
    {
        //if someone sets restore defaults on the parameter server, prevent looping
        config.restore_defaults = false;
    }

}
