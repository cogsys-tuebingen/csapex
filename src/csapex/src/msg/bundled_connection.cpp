/// HEADER
#include <csapex/msg/bundled_connection.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/input_transition.h>

using namespace csapex;

ConnectionPtr BundledConnection::connect(Output *from, Input *to, OutputTransition* ot, InputTransition* it)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to, ot, it));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr BundledConnection::connect(Output *from, Input *to, OutputTransition *ot, InputTransition *it, int id)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to, ot, it, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

BundledConnection::BundledConnection(Output *from, Input *to, OutputTransition* ot, InputTransition* it)
    : Connection(from, to), ot_(ot), it_(it)
{

}

BundledConnection::BundledConnection(Output *from, Input *to, OutputTransition* ot, InputTransition* it, int id)
    : Connection(from, to, id), ot_(ot), it_(it)
{

}

void BundledConnection::setMessage(const ConnectionTypeConstPtr &msg)
{
    Connection::setMessage(msg);

    it_->checkIfEnabled();
}


void BundledConnection::establishSource()
{
    Connection::establishSource();

    it_->checkIfEnabled();
}

void BundledConnection::establishSink()
{
    Connection::establishSink();

    it_->checkIfEnabled();
}
