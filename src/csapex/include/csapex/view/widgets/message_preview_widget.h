#ifndef MESSAGE_PREVIEW_WIDGET_H
#define MESSAGE_PREVIEW_WIDGET_H

/// COMPONENT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>

/// SYSTEM
#include <QGraphicsView>

namespace csapex
{


namespace impl {

class PreviewInput : public Input
{
public:
    PreviewInput();

    virtual void inputMessage(ConnectionType::ConstPtr message) override;
    void setCallback(std::function<void (ConnectionType::ConstPtr)> cb);

private:
    std::function<void (ConnectionType::ConstPtr)> cb_;
};

}

class MessagePreviewWidget : public QGraphicsView
{
    Q_OBJECT

public:
    MessagePreviewWidget();

public:
    void connectTo(Output* out);
    void disconnect();

    void setCallback(std::function<void(ConnectionType::ConstPtr)> cb);

    bool isConnected() const;

private:
    ConnectionPtr connection_;
    std::shared_ptr<impl::PreviewInput> input_;

    std::function<void(ConnectionType::ConstPtr)> cb_;
};

}

#endif // MESSAGE_PREVIEW_WIDGET_H
