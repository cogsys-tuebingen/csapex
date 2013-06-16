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

class Designer : public QWidget
{
    Q_OBJECT

public:
    Designer(QWidget* parent = 0);

    bool isDirty();

    bool canUndo();
    bool canRedo();

public Q_SLOTS:
    void save();
    void load();
    void undo();
    void redo();

Q_SIGNALS:
    void stateChanged();

private:
    Ui::Designer* ui;
};

}
#endif // DESIGNER_H
