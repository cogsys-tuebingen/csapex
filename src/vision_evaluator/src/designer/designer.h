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

private:
    Ui::Designer* ui;
};

}
#endif // DESIGNER_H
