#ifndef PORT_H
#define PORT_H

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/view/view_fwd.h>
#include <csapex/command/command_fwd.h>

/// SYSTEM
#include <QFrame>

namespace csapex
{

class Port : public QFrame
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

    void setPortProperty(const std::string& name, bool b);

    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

    virtual void enterEvent(QEvent* e);
    virtual void leaveEvent(QEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    virtual QPoint topLeft();
    virtual QPoint centerPoint();

    virtual bool canOutput() const;
    virtual bool canInput() const;
    virtual bool isOutput() const;
    virtual bool isInput() const;

    ConnectableWeakPtr getAdaptee() const;

    void refreshStylesheet();

Q_SIGNALS:
    void mouseOver(Port* port);
    void mouseOut(Port* port);
    void removeConnectionsRequest();

    void addConnectionRequest(Connectable*);
    void moveConnectionRequest(Connectable*);

public Q_SLOTS:
    void setMinimizedSize(bool mini);
    void setFlipped(bool flipped);
    void setEnabledFlag(bool disabled);

    void setError(bool e, const std::string& msg);
    void setError(bool e, const std::string& msg, int level);

protected:
    void startDrag();
    void createToolTip();
    void paintEvent(QPaintEvent *);

protected:
    ConnectableWeakPtr adaptee_;
    bool refresh_style_sheet_;
    bool minimized_;
    bool flipped_;

    Qt::MouseButtons buttons_down_;

    std::vector<csapex::slim_signal::Connection> connections_;
};

}

#endif // PORT_H
