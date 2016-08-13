#ifndef PORT_H
#define PORT_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/model/connectable.h>
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
    Port(ConnectableWeakPtr adaptee, QWidget *parent = nullptr);
    virtual ~Port();

    bool event(QEvent *e);

    QString cssClass() {
        return QString("Port");
    }

    bool isMinimizedSize() const;
    bool isFlipped() const;
    bool isHovered() const;

    void setPortProperty(const std::string& name, bool b);

    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent* e);
    virtual void mouseDoubleClickEvent(QMouseEvent* e);
    void mouseClickEvent();

    virtual void enterEvent(QEvent* e);
    virtual void leaveEvent(QEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    bool canOutput() const;
    bool canInput() const;
    bool isOutput() const;
    bool isInput() const;

    ConnectableWeakPtr getAdaptee() const;

    void refreshStylesheet();

Q_SIGNALS:
    void mouseOver(Port* port);
    void mouseOut(Port* port);
    void removeConnectionsRequest();

    void addConnectionRequest(Connectable*);
    void moveConnectionRequest(Connectable*);

    void changePortRequest(QString label);

public Q_SLOTS:
    void setMinimizedSize(bool mini);
    void setFlipped(bool flipped);
    void setEnabledFlag(bool disabled);

    void setError(bool e, const std::string& msg);
    void setError(bool e, const std::string& msg, int level);

protected:
    Port(QWidget *parent = nullptr);

    void startDrag();
    void createToolTip();
    void paintEvent(QPaintEvent *);

protected:
    ConnectableWeakPtr adaptee_;
    bool refresh_style_sheet_;
    bool minimized_;
    bool flipped_;

    bool hovered_;

    Qt::MouseButtons buttons_down_;

    std::vector<csapex::slim_signal::Connection> connections_;

    QTimer* double_click_timer_;
};

}

#endif // PORT_H
