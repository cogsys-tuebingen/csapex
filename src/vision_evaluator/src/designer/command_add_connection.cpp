/// HEADER
#include "command_add_connection.h"

/// COMPONENT
#include <boost/foreach.hpp>
#include "command.h"
#include "selector_proxy.h"
#include "box.h"
#include "box_manager.h"

using namespace vision_evaluator;
using namespace vision_evaluator::command;

AddConnection::AddConnection(Overlay* overlay, Connector* a, Connector* b, QWidget* parent)
    : overlay(overlay), parent(parent)
{
    from = dynamic_cast<ConnectorOut*>(a);
    if(from) {
        to = dynamic_cast<ConnectorIn*>(b);
    } else {
        from = dynamic_cast<ConnectorOut*>(b);
        to = dynamic_cast<ConnectorIn*>(a);
    }
    assert(from);
    assert(to);

    from_uuid = from->UUID();
    to_uuid = to->UUID();
}

void AddConnection::execute()
{

    if(from->tryConnect(to)) {
        overlay->addConnection(from, to);
    }
}

void AddConnection::undo()
{
    from->removeConnection(to);
    to->removeConnection(from);
    overlay->removeConnection(from, to);
}
void AddConnection::redo()
{
    // from and to might not be valid anymore
    // (if they have been deleted and restored)
    from = find<ConnectorOut>(from_uuid);
    to = find<ConnectorIn>(to_uuid);

    assert(from);
    assert(to);

    execute();
}

template <class C>
C* AddConnection::find(const std::string& uuid)
{
    QList<C*> children = parent->findChildren<C*>();
    BOOST_FOREACH(C* c, children) {
        if(c->UUID() == uuid) {
            return c;
        }
    }
    return NULL;
}
