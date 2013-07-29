/// HEADER
#include "export_cout.h"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_in.h>
#include <utils/stream_interceptor.h>
#include <vision_evaluator/messages_default.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::ExportCout, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

ExportCout::ExportCout()
    : connector_(NULL)
{
    setCategory("ConsoleIO");
    setIcon(QIcon(":/terminal.png"));
}

void ExportCout::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = new ConnectorIn(box_, 0);
        connector_->setLabel("Anything");
        connector_->setType(connection_types::AnyMessage::make());

        box_->addInput(connector_);
    }
}

void ExportCout::messageArrived(ConnectorIn *source)
{
    ConnectionType::Ptr msg = source->getMessage();

    std::cout << "writing to cout: ";
    msg->write(std::cout);
    std::cout << std::endl;

    StreamInterceptor::instance().cout << *msg << "\n---" << std::endl;
}
