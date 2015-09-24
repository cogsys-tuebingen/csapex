/// HEADER
#include <csapex/msg/bundled_connection.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/utility/assert.h>

using namespace csapex;

ConnectionPtr BundledConnection::connect(Output *from, Input *to)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr BundledConnection::connect(Output *from, Input *to, int id)
{
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

BundledConnection::BundledConnection(Output *from, Input *to)
    : Connection(from, to)
{

}

BundledConnection::BundledConnection(Output *from, Input *to, int id)
    : Connection(from, to, id)
{

}
