/// HEADER
#include "double_buffer.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>

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
        input_ = box_->addInput<connection_types::AnyMessage>("Anything");
        output_ = box_->addOutput<connection_types::AnyMessage>("Same as input");
    }
}

void DoubleBuffer::messageArrived(ConnectorIn *source)
{
    ConnectionType::Ptr msg = source->getMessage<ConnectionType>();

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
