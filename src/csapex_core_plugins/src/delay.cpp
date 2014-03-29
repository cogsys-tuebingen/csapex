/// HEADER
#include "delay.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/qt_helper.hpp>

CSAPEX_REGISTER_CLASS(csapex::Delay, csapex::Node)

using namespace csapex;

Delay::Delay()
    : input_(NULL), output_(NULL)
{
    addTag(Tag::get("Debug"));

    addParameter(param::ParameterFactory::declare<double>("delay", 0.0, 10.0, 1.0, 0.1));
}

QIcon Delay::getIcon() const
{
    return QIcon(":/buffer.png");
}

void Delay::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::AnyMessage>("Input");
    output_ = addOutput<connection_types::AnyMessage>("Delayed Input");
}

void Delay::process()
{
    ConnectionType::Ptr msg = input_->getMessage<ConnectionType>();

    long t = param<double>("delay") * 1000;
    qt_helper::QSleepThread::msleep(t);

    output_->setType(input_->getType());
    output_->publish(msg);
}

