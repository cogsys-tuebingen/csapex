#ifndef PORT_H
#define PORT_H

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/model/error_state.h>
#include <csapex/view/selectable.h>

/// SYSTEM
#include <QFrame>

namespace csapex
{

class Port : public QFrame, public Selectable/*, public ErrorState*/
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    Port(CommandDispatcher* dispatcher, Connectable* adaptee);
    virtual ~Port();

    QString cssClass() {
        return QString("Port");
    }

    bool isMinimizedSize() const;
    bool isFlipped() const;

    void setPortProperty(const std::string& name, bool b);

    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    virtual QPoint topLeft();
    virtual QPoint centerPoint();

    virtual bool canOutput() const;
    virtual bool canInput() const;
    virtual bool isOutput() const;
    virtual bool isInput() const;

    Connectable* getAdaptee() const;

    void refreshStylesheet();

public Q_SLOTS:
    void setMinimizedSize(bool mini);
    void setFlipped(bool flipped);
    void setBlocked(bool blocked);
    void setEnabledFlag(bool disabled);

    void setError(bool e, const std::string& msg);
    void setError(bool e, const std::string& msg, int level);

protected:
    void createToolTip();
    void paintEvent(QPaintEvent *);

protected:
    CommandDispatcher* dispatcher_;
    Connectable * adaptee_;
    bool refresh_style_sheet_;
    bool minimized_;
    bool flipped_;

    Qt::MouseButtons buttons_down_;

public:
    mutable long guard_;
};

}

#endif // PORT_H
