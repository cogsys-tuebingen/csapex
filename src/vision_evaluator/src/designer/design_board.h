#ifndef DESIGN_BOARD_H
#define DESIGN_BOARD_H

/// COMPONENT
#include "overlay.h"
#include "connector.h"

/// SYSTEM
#include <QWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class DesignBoard;
}

namespace vision_evaluator
{

class DesignBoard : public QWidget
{
    Q_OBJECT

public:
    DesignBoard(QWidget* parent = 0);
    virtual ~DesignBoard();

    void resizeEvent(QResizeEvent* e);
    void childEvent(QChildEvent* e);

    virtual bool eventFilter(QObject* o, QEvent* e);
    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

public Q_SLOTS:
    void showContextMenu(const QPoint& pos);

private:
    Ui::DesignBoard* ui;

    Overlay* overlay;
};

}
#endif // DESIGN_BOARD_H
