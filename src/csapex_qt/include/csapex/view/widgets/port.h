#ifndef PORT_H
#define PORT_H

/// COMPONENT
#include <csapex_qt/export.h>

/// PROJECT
#include <csapex/model/connector.h>
#include <csapex/view/view_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/model/connector_type.h>

/// SYSTEM
#include <QFrame>

namespace csapex
{
class CSAPEX_QT_EXPORT Port : public QFrame
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    Port(ConnectorPtr adaptee, QWidget* parent = nullptr);
    virtual ~Port();

    bool event(QEvent* e);

    QString cssClass()
    {
        return QString("Port");
    }

    bool isMinimizedSize() const;
    bool isFlipped() const;
    bool isHovered() const;

    void setPortProperty(const std::string& name, bool b);

    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);
    virtual void mouseDoubleClickEvent(QMouseEvent* e);
    void mouseClickEvent();

    virtual void enterEvent(QEvent* e);
    virtual void leaveEvent(QEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    bool isOutput() const;
    bool isInput() const;

    ConnectorPtr getAdaptee() const;

    void refreshStylesheet();

Q_SIGNALS:
    void mouseOver(Port* port);
    void mouseOut(Port* port);
    void removeConnectionsRequest();

    void addConnectionPreview(ConnectorPtr);
    void addConnectionRequest(ConnectorPtr);

    void moveConnectionPreview(ConnectorPtr);
    void moveConnectionRequest(ConnectorPtr);

    void changePortRequest(QString label);

public Q_SLOTS:
    void setMinimizedSize(bool mini);
    void setFlipped(bool flipped);
    void setEnabledFlag(bool disabled);

protected:
    Port(QWidget* parent = nullptr);

    void startDrag();
    virtual void createToolTip();
    void paintEvent(QPaintEvent*);

protected:
    ConnectorPtr adaptee_;
    bool refresh_style_sheet_;
    bool minimized_;
    bool flipped_;

    bool hovered_;

    Qt::MouseButtons buttons_down_;

    std::vector<csapex::slim_signal::Connection> connections_;

    QTimer* double_click_timer_;
};

}  // namespace csapex

#endif  // PORT_H
