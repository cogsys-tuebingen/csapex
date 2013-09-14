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

public Q_SLOTS:
    void addBox(Box* box);
    void deleteBox(Box* box);
    void stateChangedEvent();
    void reloadBoxMenues();

    void enableGrid(bool);

private:
    Ui::Designer* ui;
    DesignBoard* designer_board;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;

    QTreeWidget* menu;
};

}
#endif // DESIGNER_H
