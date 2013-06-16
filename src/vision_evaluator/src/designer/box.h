#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include "connector_in.h"
#include "connector_out.h"
#include "memento.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class Box;
}


namespace vision_evaluator
{

class BoxedObject;

class Box : public QWidget
{
    Q_OBJECT

public:
    struct MoveOffset : public QObjectUserData {
        MoveOffset(const QPoint& o)
            : value(o)
        {}

        QPoint value;
    };

public:
    static const QString MIME;
    static const QString MIME_MOVE;


public:
    Box(BoxedObject* content, const std::string& uuid = "", QWidget* parent = 0);
    virtual ~Box();

    void stop();

    virtual void mousePressEvent(QMouseEvent* e);
    virtual QPixmap makePixmap(const std::string& label);

    void moveEvent(QMoveEvent *);

    virtual void init(const QPoint& pos);

    void addInput(ConnectorIn* in);
    void addOutput(ConnectorOut* out);

    void setUUID(const std::string& uuid);
    std::string UUID() const;

    Memento::Ptr saveState();
    void loadState(Memento::Ptr state);

protected:
    void startDrag(QPoint offset);
    void paintEvent(QPaintEvent* e);
    bool eventFilter(QObject*, QEvent*);

public Q_SLOTS:
    virtual void setOverlay(Overlay* o);
    void showContextMenu(const QPoint& pos);

Q_SIGNALS:
    void toggled(bool);

private:
    Ui::Box* ui;

    std::vector<ConnectorIn*> input;
    std::vector<ConnectorOut*> output;

    bool down_;

    BoxedObject* content_;
    Overlay* overlay_;

    std::string uuid_;
};

}
#endif // BOX_H
