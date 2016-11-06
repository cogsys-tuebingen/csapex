/// HEADER
#include <csapex/msg/direct_connection.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

ConnectionPtr DirectConnection::connect(OutputPtr from, InputPtr to)
{
    apex_assert_hard(from->isConnectionPossible(to.get()));
    ConnectionPtr r(new DirectConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr DirectConnection::connect(OutputPtr from, InputPtr to, int id)
{
    apex_assert_hard(from->isConnectionPossible(to.get()));
    ConnectionPtr r(new DirectConnection(from, to, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

DirectConnection::~DirectConnection()
{
}


DirectConnection::DirectConnection(OutputPtr from, InputPtr to)
    : Connection(from, to)
{

}

DirectConnection::DirectConnection(OutputPtr from, InputPtr to, int id)
    : Connection(from, to, id)
{

}

void DirectConnection::setToken(const TokenPtr &msg)
{
    Connection::setToken(msg);

//    dynamic_cast<Input*>(to())->inputMessage(msg);

//    setState(Connection::State::READ);
////    setState(Connection::State::DONE);

//    setMessageProcessed();
}
