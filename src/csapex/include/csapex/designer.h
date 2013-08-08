#ifndef DESIGNER_H
#define DESIGNER_H

/// SYSTEM
#include <QWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class Designer;
}

namespace csapex
{

class DesignBoard;
class Box;
class Graph;

class Designer : public QWidget
{
    Q_OBJECT

    friend class DesignerIO;

public:
    Designer(csapex::Graph &graph, QWidget* parent = 0);
    virtual ~Designer();

    bool eventFilter(QObject* o, QEvent* e);
    void resizeEvent(QResizeEvent* e);

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void updateCursor();

public Q_SLOTS:
    void addBox(Box* box);
    void deleteBox(Box* box);
    void stateChangedEvent();

private:
    Ui::Designer* ui;
    DesignBoard* designer_board;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;
};

}
#endif // DESIGNER_H
