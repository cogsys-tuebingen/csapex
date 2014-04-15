/// HEADER
#include "double_buffer.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::DoubleBuffer, csapex::Node)

using namespace csapex;

DoubleBuffer::DoubleBuffer()
    : input_(NULL), output_(NULL), dirty_(false)
{
    addTag(Tag::get("Buffer"));
    addTag(Tag::get("General"));


    addParameter(param::ParameterFactory::declareBool("synchronized", true));
}

QIcon DoubleBuffer::getIcon() const
{
    return QIcon(":/buffer.png");
}

void DoubleBuffer::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::AnyMessage>("Anything");
    output_ = addOutput<connection_types::AnyMessage>("Same as input");
    output_->setAsync(true);
}

void DoubleBuffer::checkIfDone()
{
    Node::checkIfDone();
    //input_->setProcessing(false);
}

void DoubleBuffer::process()
{
    ConnectionType::Ptr msg = input_->getMessage<ConnectionType>();

    buffer_back_ = msg->clone();

    swapBuffers();

    if(output_->getType()->name() != msg->name()) {
        output_->setType(msg->toType());
    }

    dirty_ = true;
}

void DoubleBuffer::swapBuffers()
{
    QMutexLocker lock(&mutex_);

    buffer_front_ = buffer_back_;
    buffer_back_.reset();
}

void DoubleBuffer::tick()
{
    if(!dirty_ && param<bool>("synchronized")) {
        return;
    }

    if(!buffer_front_) {
        return;
    }

    ConnectionType::Ptr msg;
    {
        QMutexLocker lock(&mutex_);
        msg = buffer_front_;
    }

    output_->publish(msg);
    dirty_ = false;
}
