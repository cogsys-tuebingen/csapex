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
    Box(BoxedObject* content, QWidget* parent = 0);
    virtual ~Box();

    virtual void mousePressEvent(QMouseEvent* e);
    virtual QPixmap makePixmap(const std::string &label);

    virtual void init(const QPoint &pos);

    void addInput(ConnectorIn* in);
    void addOutput(ConnectorOut* out);

protected:
    void startDrag(QPoint offset);
    void paintEvent(QPaintEvent * e);
    bool eventFilter(QObject *, QEvent *);

public Q_SLOTS:
    virtual void setOverlay(Overlay* o);
    void showContextMenu(const QPoint& pos);

private:
    Ui::Box* ui;

    std::vector<ConnectorIn*> input;
    std::vector<ConnectorOut*> output;

    bool down_;

    BoxedObject* content_;
    Overlay* overlay_;
};

}
#endif // BOX_H
