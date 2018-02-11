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
    apex_assert_hard(from);
    apex_assert_hard(to);
    if(!Connection::canBeConnectedTo(from.get(), to.get())) {
        return nullptr;
    }
    ConnectionPtr r(new DirectConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr DirectConnection::connect(OutputPtr from, InputPtr to, int id)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    if(!Connection::canBeConnectedTo(from.get(), to.get())) {
        return nullptr;
    }
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
}
