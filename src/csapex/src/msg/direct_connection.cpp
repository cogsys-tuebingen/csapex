/// HEADER
#include <csapex/msg/direct_connection.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/utility/assert.h>

using namespace csapex;

ConnectionPtr DirectConnection::connect(Output *from, Input *to)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new DirectConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr DirectConnection::connect(Output *from, Input *to, int id)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new DirectConnection(from, to, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

DirectConnection::~DirectConnection()
{
}


DirectConnection::DirectConnection(Output *from, Input *to)
    : Connection(from, to)
{

}

DirectConnection::DirectConnection(Output *from, Input *to, int id)
    : Connection(from, to, id)
{

}

void DirectConnection::setMessage(const ConnectionTypeConstPtr &msg)
{
    Connection::setMessage(msg);

    dynamic_cast<Input*>(to())->inputMessage(msg);

    setState(Connection::State::READ);
//    setState(Connection::State::DONE);

    setMessageProcessed();
}
