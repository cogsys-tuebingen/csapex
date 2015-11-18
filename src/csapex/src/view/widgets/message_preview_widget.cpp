/// HEADER
#include <csapex/view/widgets/message_preview_widget.h>

/// PROJECT
#include <csapex/msg/direct_connection.h>

using namespace csapex;
using namespace csapex::impl;

PreviewInput::PreviewInput()
    : Input(UUID::make("message_preview_in"))
{
    setType(std::make_shared<connection_types::AnyMessage>());
}

void PreviewInput::inputMessage(ConnectionType::ConstPtr message)
{
    Input::inputMessage(message);

    cb_(message);

    notifyMessageProcessed();
}

void PreviewInput::setCallback(std::function<void (ConnectionType::ConstPtr)> cb)
{
    cb_ = cb;
}

MessagePreviewWidget::MessagePreviewWidget()
{
    input_ = std::make_shared<impl::PreviewInput>();
}

void MessagePreviewWidget::connectTo(Output *out)
{
    connection_ = DirectConnection::connect(out, input_.get());
    connection_->establishSink();
}

void MessagePreviewWidget::disconnect()
{
    if(!connection_) {
        return;
    }
    Output* out = dynamic_cast<Output*> (connection_->from());
    if(out) {
        out->removeConnection(input_.get());
    }
    connection_.reset();
}

void MessagePreviewWidget::setCallback(std::function<void (ConnectionType::ConstPtr)> cb)
{
    cb_ = cb;
    if(input_) {
        input_->setCallback(cb_);
    }
}

bool MessagePreviewWidget::isConnected() const
{
    return static_cast<bool>(connection_);
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_message_preview_widget.cpp"
