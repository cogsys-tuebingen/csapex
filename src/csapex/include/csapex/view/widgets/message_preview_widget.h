#ifndef MESSAGE_PREVIEW_WIDGET_H
#define MESSAGE_PREVIEW_WIDGET_H

/// COMPONENT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>

/// SYSTEM
#include <QGraphicsView>

namespace csapex
{

class MessagePreviewWidget;

namespace impl {
class PreviewInput : public Input
{
public:
    PreviewInput(MessagePreviewWidget* parent);

    virtual void inputMessage(Token::ConstPtr message) override;

    virtual bool isVirtual() const {
        return true;
    }

private:
    MessagePreviewWidget* parent_;
};
}

class MessagePreviewWidget : public QGraphicsView
{
    Q_OBJECT

public:
    MessagePreviewWidget();
    ~MessagePreviewWidget();

public:
    void connectTo(Connectable* c);
    void disconnect();

    void setCallback(std::function<void(Token::ConstPtr)> cb);


    bool isConnected() const;

Q_SIGNALS:
    void displayImageRequest(const QImage& msg);
    void displayTextRequest(const QString &txt);

public Q_SLOTS:
    void displayImage(const QImage& msg);
    void displayText(const QString &txt);
    
private:
    void connectToImpl(Output* out);
    void connectToImpl(Input* out);

private:
    ConnectionPtr connection_;
    std::shared_ptr<impl::PreviewInput> input_;

    QString displayed_;

    QGraphicsPixmapItem* pm_item_;
    QGraphicsTextItem* txt_item_;
};

}

#endif // MESSAGE_PREVIEW_WIDGET_H
