#ifndef MESSAGE_PREVIEW_WIDGET_H
#define MESSAGE_PREVIEW_WIDGET_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>

/// SYSTEM
#include <QGraphicsView>
#include <QPointer>

namespace csapex
{

class MessagePreviewWidget;

namespace impl {
class CSAPEX_QT_EXPORT PreviewInput : public Input
{
public:
    PreviewInput(QPointer<MessagePreviewWidget> parent);

    virtual void setToken(TokenPtr message) override;

    virtual bool isVirtual() const {
        return true;
    }

    void detach();

private:
    QPointer<MessagePreviewWidget> parent_;
};
}


class CSAPEX_QT_EXPORT MessagePreviewWidget : public QGraphicsView
{
    Q_OBJECT

public:
    MessagePreviewWidget();
    ~MessagePreviewWidget();

public:
    void connectTo(ConnectablePtr c);
    void disconnect();

    void setCallback(std::function<void(TokenData::ConstPtr)> cb);


    bool isConnected() const;

Q_SIGNALS:
    void displayImageRequest(const QImage& msg);
    void displayTextRequest(const QString &txt);

public Q_SLOTS:
    void displayImage(const QImage& msg);
    void displayText(const QString &txt);
    
private:
    void connectToImpl(OutputPtr out);
    void connectToImpl(InputPtr out);

private:
    ConnectionPtr connection_;
    std::shared_ptr<impl::PreviewInput> input_;

    QString displayed_;

    QGraphicsPixmapItem* pm_item_;
    QGraphicsTextItem* txt_item_;
};

}

#endif // MESSAGE_PREVIEW_WIDGET_H
