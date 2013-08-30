/// HEADER
#include "double_buffer.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/connection_type.h>
#include <csapex/message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::DoubleBuffer, csapex::BoxedObject)

using namespace csapex;

DoubleBuffer::DoubleBuffer()
    : input_(NULL), output_(NULL)
{
    addTag(Tag::get("Buffer"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/buffer.png"));
}

void DoubleBuffer::fill(QBoxLayout *layout)
{
    if(input_ == NULL) {
        input_ = new ConnectorIn(box_, 0);
        input_->setLabel("Anything");
        input_->setType(connection_types::AnyMessage::make());

        output_ = new ConnectorOut(box_, 0);
        output_->setLabel("Same as input");
        output_->setType(connection_types::AnyMessage::make());

        box_->addInput(input_);
        box_->addOutput(output_);
    }
}

void DoubleBuffer::messageArrived(ConnectorIn *source)
{
    ConnectionType::Ptr msg = source->getMessage();

    state.buffer_back_ = msg->clone();

    swapBuffers();

    if(output_->getType()->name() != msg->name()) {
        std::cout << "changing type to " << msg->name() << std::endl;
        output_->setType(msg->toType());
    }

    msg->write(std::cout);
    std::cout << std::endl;
}

void DoubleBuffer::swapBuffers()
{
    QMutexLocker lock(&mutex_);

    state.buffer_front_ = state.buffer_back_;
    state.buffer_back_.reset();
}

void DoubleBuffer::tick()
{
    if(!state.buffer_front_) {
        return;
    }

    ConnectionType::Ptr msg;
    {
        QMutexLocker lock(&mutex_);
        msg = state.buffer_front_;
    }

    output_->publish(msg);
}


Memento::Ptr DoubleBuffer::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void DoubleBuffer::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;
}
