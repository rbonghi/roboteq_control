
#include "configurator/motor_param.h"


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

    ds_param = new dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<roboteq_control::RoboteqParameterConfig>::CallbackType cb_param = boost::bind(&MotorParamConfigurator::reconfigureCBParam, this, _1, _2);
    ds_param->setCallback(cb_param);
}

motor_params_t MotorParamConfigurator::initConfigurator() {

//    int temp_int;
    double temp_double;
    nh_.getParam(mName + "/ratio", temp_double);
    params.ratio = (float) temp_double;
//    nh_.getParam(mName + "/rotation", temp_int);
//    parameter.rotation = (int8_t) temp_int;

    // Load parameters from Roboteq
    getParam();


    return params;
}

void MotorParamConfigurator::getParam() {
    try
    {
        // Get Encoder PPR - Will be move in Encoder page
        if(mSerial->query("EPPR", std::to_string(mNumber), "~"))
        {
            params.ppr = boost::lexical_cast<unsigned int>(mSerial->get());

        }
        // Get Motor direction
        if(mSerial->query("MDIR", std::to_string(mNumber), "~"))
        {
            int sign = boost::lexical_cast<int>(mSerial->get());
            params.direction = sign ? 1 : -1;
        }
        // Get Max RPM motor
        if(mSerial->query("MXRPM", std::to_string(mNumber), "~"))
        {
            params.max_rpm = boost::lexical_cast<unsigned int>(mSerial->get());
        }
        // ROS_INFO_STREAM("[" << mNumber << "]" << "R=" << params.ratio << " PPR=" << params.ppr << " MDIR=" << params.direction << " MXRPM=" << params.max_rpm);

    } catch (std::bad_cast& e)
    {
        ROS_WARN_STREAM("Failure parsing feedback data. Dropping message." << e.what());
    }
}

void MotorParamConfigurator::reconfigureCBParam(roboteq_control::RoboteqParameterConfig &config, uint32_t level) {

}
