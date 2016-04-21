/// HEADER
#include <csapex/signal/signal_connection.h>

/// PROJECT
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/utility/assert.h>

using namespace csapex;

ConnectionPtr SignalConnection::connect(Event *from, Slot *to)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new SignalConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr SignalConnection::connect(Event *from, Slot *to, int id)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new SignalConnection(from, to, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

SignalConnection::SignalConnection(Event *from, Slot *to)
    : Connection(from, to)
{
    from->connect(to);
    establishSource();
    establishSink();
}

SignalConnection::SignalConnection(Event *from, Slot *to, int /*id*/)
    : Connection(from, to)
{
    from->connect(to);
    establishSource();
    establishSink();
}

