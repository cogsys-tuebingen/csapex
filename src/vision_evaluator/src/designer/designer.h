#ifndef DESIGNER_H
#define DESIGNER_H

/// SYSTEM
#include <QWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class Designer;
}

namespace vision_evaluator
{

class Box;

class Designer : public QWidget
{
    Q_OBJECT

    friend class DesignerIO;

public:
    Designer(QWidget* parent = 0);

    bool eventFilter(QObject* o, QEvent* e);
    void resizeEvent(QResizeEvent* e);

    bool isDirty();

    bool canUndo();
    bool canRedo();

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void updateCursor();

public Q_SLOTS:
    void save();
    void load();
    void undo();
    void redo();
    void clear();

Q_SIGNALS:
    void stateChanged();

private:
    Ui::Designer* ui;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;
};

}
#endif // DESIGNER_H
