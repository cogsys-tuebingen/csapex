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
    ConnectionPtr r(new BundledConnection(from, to, ot, it));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}
ConnectionPtr BundledConnection::connect(Output *from, Input *to, OutputTransition *ot, InputTransition *it, int id)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to, ot, it, id));
    from->addConnection(r);
    to->addConnection(r);
    return r;
}

ConnectionPtr BundledConnection::connect(Output *from, Input *to, OutputTransition* ot)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to, ot, nullptr));
    from->addConnection(r);
    to->addConnection(r);

    // no input transition to establish
    r->establishSink();
    return r;
}

ConnectionPtr BundledConnection::connect(Output *from, Input *to, InputTransition* it)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard(from->isConnectionPossible(to));
    ConnectionPtr r(new BundledConnection(from, to, nullptr, it));
    from->addConnection(r);
    to->addConnection(r);

    // no output transition to establish
    r->establishSource();
    return r;
}

BundledConnection::BundledConnection(Output *from, Input *to, OutputTransition* ot, InputTransition* it)
    : DirectConnection(from, to), ot_(ot), it_(it)
{

}

BundledConnection::BundledConnection(Output *from, Input *to, OutputTransition* ot, InputTransition* it, int id)
    : DirectConnection(from, to, id), ot_(ot), it_(it)
{

}

void BundledConnection::setMessage(const ConnectionTypeConstPtr &msg)
{

    if(it_) {
        Connection::setMessage(msg);
    } else {
        DirectConnection::setMessage(msg);
    }

    if(it_) {
        it_->checkIfEnabled();
    }

    if(ot_) {
        ot_->checkIfEnabled();
    }
}


void BundledConnection::establishSource()
{
    Connection::establishSource();

    if(it_) {
        it_->checkIfEnabled();
    }

    if(ot_) {
        ot_->checkIfEnabled();
    }
}

void BundledConnection::establishSink()
{
    Connection::establishSink();

    if(it_) {
        it_->checkIfEnabled();
    }

    if(ot_) {
        ot_->checkIfEnabled();
    }
}
