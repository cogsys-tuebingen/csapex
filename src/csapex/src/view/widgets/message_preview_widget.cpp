/// HEADER
#include <csapex/view/widgets/message_preview_widget.h>

/// PROJECT
#include <csapex/msg/direct_connection.h>
#include <csapex/manager/message_renderer_manager.h>

/// SYSTEM
#include <QApplication>

using namespace csapex;
using namespace csapex::impl;

PreviewInput::PreviewInput(MessagePreviewWidget *parent)
    : Input(UUID::make_forced("message_preview_in")),
      parent_(parent)
{
    setType(std::make_shared<connection_types::AnyMessage>());
}

void PreviewInput::inputMessage(ConnectionType::ConstPtr message)
{
    Input::inputMessage(message);

    Q_EMIT parent_->displayMessage(message);

    notifyMessageProcessed();
}


MessagePreviewWidget::MessagePreviewWidget()
{
    input_ = std::make_shared<impl::PreviewInput>(this);
    setScene(new QGraphicsScene);

    qRegisterMetaType < ConnectionTypeConstPtr > ("ConnectionTypeConstPtr");

    QObject::connect(this, SIGNAL(displayMessage(ConnectionTypeConstPtr)), this, SLOT(display(ConnectionTypeConstPtr)));
}

MessagePreviewWidget::~MessagePreviewWidget()
{
    if(isConnected()) {
        disconnect();
    }
}

void MessagePreviewWidget::connectTo(Connectable *c)
{
    scene()->clear();

    if(Output* o = dynamic_cast<Output*>(c)) {
        connectToImpl(o);
    } else if(Input* i = dynamic_cast<Input*>(c)) {
        connectToImpl(i);
    } else {
        return;
    }

    if(connection_) {
        connection_->establishSink();
        QApplication::setOverrideCursor(Qt::BusyCursor);
    }
}

void MessagePreviewWidget::connectToImpl(Output *out)
{
    connection_ = DirectConnection::connect(out, input_.get());
}


void MessagePreviewWidget::connectToImpl(Input *in)
{
    if(in->isConnected()) {
        if(Output* source = dynamic_cast<Output*>(in->getSource())) {
            connection_ = DirectConnection::connect(source, input_.get());
        }
    }
}

void MessagePreviewWidget::disconnect()
{
    if(!connection_) {
        return;
    }

    while(QApplication::overrideCursor()) {
        QApplication::restoreOverrideCursor();
    }

    Output* out = dynamic_cast<Output*> (connection_->from());
    if(out) {
        out->removeConnection(input_.get());
    }
    connection_.reset();
}

void MessagePreviewWidget::display(const ConnectionTypeConstPtr &msg)
{
    if(isConnected()) {
        try {
            MessageRenderer::Ptr renderer = MessageRendererManager::instance().createMessageRenderer(msg);
            if(renderer) {
                QImage img = renderer->render(msg);


                auto pm = QPixmap::fromImage(img);
                scene()->clear();
                scene()->addPixmap(pm);
                setMaximumSize(256, 256);
                fitInView(scene()->sceneRect(), Qt::KeepAspectRatio);

                show();
                return;
            }
        } catch(const std::exception& e) {
            // silent death
        }
    }

    hide();
}

bool MessagePreviewWidget::isConnected() const
{
    return static_cast<bool>(connection_);
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_message_preview_widget.cpp"