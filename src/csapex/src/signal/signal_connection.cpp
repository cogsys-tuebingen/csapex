/// HEADER
#include <csapex/signal/signal_connection.h>

/// PROJECT
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/utility/assert.h>

using namespace csapex;

ConnectionPtr SignalConnection::connect(Trigger *from, Slot *to)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new SignalConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr SignalConnection::connect(Trigger *from, Slot *to, int id)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new SignalConnection(from, to, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

SignalConnection::SignalConnection(Trigger *from, Slot *to)
    : Connection(from, to)
{
    from->connect(to);
}

SignalConnection::SignalConnection(Trigger *from, Slot *to, int /*id*/)
    : Connection(from, to)
{
    from->connect(to);
}

