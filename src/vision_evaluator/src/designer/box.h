#ifndef BOX_H
#define BOX_H

/// COMPONENT
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

class Box : public QWidget
{
    Q_OBJECT

public:
    struct MoveOffset : public QObjectUserData
    {
        MoveOffset(const QPoint& o)
            : value(o)
        {}

        QPoint value;
    };

public:
    static const QString MIME;
    static const QString MIME_MOVE;


public:
    Box(QWidget* parent = 0);
    virtual ~Box();

    virtual void mousePressEvent(QMouseEvent * e);
    virtual QPixmap makePixmap();

public Q_SLOTS:
    virtual void setOverlay(Overlay *o);
    void showContextMenu(const QPoint& pos);

private:
    Ui::Box* ui;

    ConnectorIn* input;
    ConnectorOut* output;

    Overlay* overlay_;
};

}
#endif // BOX_H
