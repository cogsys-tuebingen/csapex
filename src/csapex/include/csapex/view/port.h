#ifndef PORT_H
#define PORT_H

/// PROJECT
#include <csapex/model/connectable.h>

/// SYSTEM
#include <QFrame>

namespace csapex
{

class Port : public QFrame
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    Port(CommandDispatcher* dispatcher, WidgetController *widget_controller, ConnectablePtr adaptee, QWidget *parent = nullptr);
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

public Q_SLOTS:
    void setMinimizedSize(bool mini);
    void setFlipped(bool flipped);
    void setEnabledFlag(bool disabled);

    void setError(bool e, const std::string& msg);
    void setError(bool e, const std::string& msg, int level);

    void updateTooltip();

protected:
    void startDrag();
    void createToolTip();
    void paintEvent(QPaintEvent *);

protected:
    CommandDispatcher* dispatcher_;
    WidgetController *widget_controller_;
    ConnectableWeakPtr adaptee_;
    bool refresh_style_sheet_;
    bool minimized_;
    bool flipped_;

    Qt::MouseButtons buttons_down_;

    boost::signals2::connection tooltip_connection;
    std::vector<boost::signals2::connection> connections_;

public:
    mutable long guard_;
};

}

#endif // PORT_H
