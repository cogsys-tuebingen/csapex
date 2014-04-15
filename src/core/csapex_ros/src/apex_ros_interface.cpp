/// HEADER
#include "apex_ros_interface.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/ros_handler.h>

CSAPEX_REGISTER_CLASS(csapex::APEXRosInterface, csapex::CorePlugin)

using namespace csapex;

APEXRosInterface::APEXRosInterface()
    : core_(NULL)
{
}

APEXRosInterface::~APEXRosInterface()
{
}

void APEXRosInterface::init(CsApexCore &core)
{
    core_ = &core;

    ROSHandler::instance().waitForConnection();

    if(ROSHandler::instance().isConnected()) {
        std::cerr << "subscribing to /syscommand" << std::endl;
        command_sub_ = ROSHandler::instance().nh()->subscribe
                <std_msgs::String>("/syscommand", 10, boost::bind(&APEXRosInterface::command, this, _1));

        ros::spinOnce();

    } else {
        std::cerr << "cannot init ros interface, no ros handler" << std::endl;
    }
}

void APEXRosInterface::command(const std_msgs::StringConstPtr& cmd)
{
    std::string command = cmd->data;

    if(command == "pause") {
        core_->setPause(true);

    } else if(command == "unpause" || command == "continue" || command == "play"){
        core_->setPause(false);
    }
}
