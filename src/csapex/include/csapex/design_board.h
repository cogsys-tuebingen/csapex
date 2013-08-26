#ifndef DESIGN_BOARD_H
#define DESIGN_BOARD_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class DesignBoard;
}

namespace csapex
{

class DesignBoard : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(QString class READ cssClass)

public:
    DesignBoard(QWidget* parent = 0);
    virtual ~DesignBoard();

    void paintEvent(QPaintEvent*);
    void resizeEvent(QResizeEvent* e);

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void mousePressEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);

    virtual bool eventFilter(QObject* o, QEvent* e);
    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

    QString cssClass() {
        return QString("DesignBoard");
    }

public Q_SLOTS:
    void updateCursor();
    void showContextMenu(const QPoint& pos);
    void findMinSize(Box* box);

    void addBoxEvent(Box* box);
    void refresh();

private:
    Ui::DesignBoard* ui;

    Overlay* overlay;
    DragIO& drag_io;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;
};

}
#endif // DESIGN_BOARD_H
