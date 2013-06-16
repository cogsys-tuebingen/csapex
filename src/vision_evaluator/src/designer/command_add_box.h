#ifndef COMMAND_ADD_BOX_HPP
#define COMMAND_ADD_BOX_HPP

/// COMPONENT
#include "command.h"
#include "memento.h"

/// SYSTEM
#include <QWidget>

namespace vision_evaluator {

class SelectorProxy;
class Box;

namespace command {

struct AddBox : public Command {
    AddBox(SelectorProxy* selector, QWidget* parent, QPoint pos);

protected:
    void execute();
    void undo();
    void redo();

private:
    SelectorProxy* selector;
    QWidget* parent;
    QPoint pos;

    std::string uuid;

    vision_evaluator::Box* box;

    Memento::Ptr saved_state;
};
}
}

#endif // COMMAND_ADD_BOX_HPP
