#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include "drag_tracker.h"
#include "connector_in.h"
#include "connector_out.h"

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

class Box : public QWidget, public DragTracker
{
    Q_OBJECT

    virtual void mousePressEvent(QMouseEvent * e) {DragTracker::mousePressEvent(e);}
    virtual void mouseMoveEvent(QMouseEvent* e) {DragTracker::mouseMoveEvent(e);}

public:
    Box(QWidget* parent = 0);
    virtual ~Box();

    virtual void mouseReleaseEvent(QMouseEvent * e);

    virtual QPixmap makePixmap();

public Q_SLOTS:
    virtual void setOverlay(Overlay *o);
    void showContextMenu(const QPoint& pos);

protected:
    void mouseDelta(const QPoint& delta);
    virtual QPoint getPos() const;

private:
    Ui::Box* ui;

    ConnectorIn* input;
    ConnectorOut* output;
};

}
#endif // BOX_H
