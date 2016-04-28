/// HEADER
#include <csapex/msg/bundled_connection.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>

using namespace csapex;

ConnectionPtr BundledConnection::connect(Output *from, Input *to, OutputTransition* ot, InputTransition* it)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr BundledConnection::connect(Output *from, Input *to, OutputTransition *ot, InputTransition *it, int id)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

ConnectionPtr BundledConnection::connect(Output *from, Input *to, OutputTransition* ot)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);

    return r;
}

ConnectionPtr BundledConnection::connect(Output *from, Input *to, InputTransition* it)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to));
    from->addConnection(r);
    to->addConnection(r);

    return r;
}

BundledConnection::BundledConnection(Output *from, Input *to)
    : DirectConnection(from, to)
{

}

BundledConnection::BundledConnection(Output *from, Input *to, int id)
    : DirectConnection(from, to, id)
{

}
