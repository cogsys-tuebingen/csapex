#ifndef DESIGNER_H
#define DESIGNER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>
#include <QTreeWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class Designer;
}

namespace csapex
{

class Designer : public QWidget
{
    Q_OBJECT

    friend class DesignerIO;

public:
    Designer(CommandDispatcher* dispatcher, QWidget* parent = 0);
    virtual ~Designer();

    bool eventFilter(QObject* o, QEvent* e);
    void resizeEvent(QResizeEvent* e);

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void updateCursor();

    void setView(int x, int y);

public Q_SLOTS:
    void addBox(Box* box);
    void removeBox(Box* box);
    void stateChangedEvent();
    void reloadBoxMenues();
    void updateDebugInfo();
    void updateUndoInfo();

    void enableGrid(bool);
    void reset();

private:
    Ui::Designer* ui;
    DesignBoard* designer_board;

    CommandDispatcher* dispatcher_;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;

    QTreeWidget* box_selection_menu;

    QTabWidget* debug_tabs;
    QTreeWidget* box_info;
    QTreeWidget* undo_stack;
    QTreeWidget* redo_stack;

    bool is_init_;
};

}
#endif // DESIGNER_H
