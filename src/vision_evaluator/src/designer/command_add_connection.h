#ifndef COMMAND_ADD_CONNECTION_HPP
#define COMMAND_ADD_CONNECTION_HPP

/// COMPONENT
#include "command.h"

/// SYSTEM
#include <QWidget>

namespace vision_evaluator
{

class SelectorProxy;
class Box;
class Overlay;
class Connector;
class ConnectorIn;
class ConnectorOut;

namespace command
{

struct AddConnection : public Command {
    AddConnection(Overlay* overlay, Connector* a, Connector* b, QWidget* parent);

protected:
    void execute();
    void undo();
    void redo();

    template <class C>
    C* find(const std::string& uuid);

private:
    Overlay* overlay;
    ConnectorOut* from;
    ConnectorIn* to;
    QWidget* parent;

    std::string from_uuid;
    std::string to_uuid;
};
}
}
#endif // COMMAND_ADD_CONNECTION_HPP
