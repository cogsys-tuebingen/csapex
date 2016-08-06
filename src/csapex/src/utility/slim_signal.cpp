/// HEADER
#include <csapex/utility/slim_signal.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <algorithm>

using namespace csapex;
using namespace slim_signal;

SignalBase::SignalBase()
    : guard_(-1)
{

}

SignalBase::~SignalBase()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    while(!connections_.empty()) {
        Connection* c = connections_.front();
        apex_assert_hard(c->parent_ == this);
        c->detach();
    }

    guard_ = 0xDEADBEEF;
}

void SignalBase::addConnection(Connection *connection)
{
    apex_assert_hard(connection->parent_ == this);
    apex_assert_hard(guard_ == -1);

    std::unique_lock<std::recursive_mutex> lock(mutex_);

    connections_.push_back(connection);
}

void SignalBase::removeConnection(const Connection *connection)
{
    apex_assert_hard(connection->parent_ == this);
    apex_assert_hard(guard_ == -1);

    std::unique_lock<std::recursive_mutex> lock(mutex_);

    for(auto it = connections_.begin(); it != connections_.end();) {
        if(*it == connection) {
            it = connections_.erase(it);
        } else {
            ++it;
        }
    }
}

void SignalBase::disconnectAll()
{
    apex_assert_hard(guard_ == -1);
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    for(Connection* c : connections_) {
        c->disconnect();
    }
    connections_.clear();
}


Connection::Connection(SignalBase* parent, const Deleter& del)
    : parent_(parent), deleter_(del)
{
    apex_assert_hard(parent);
    parent_->addConnection(this);
}

Connection::Connection(const Connection& other)
    : parent_(other.parent_), deleter_(other.deleter_)
{
    if(parent_) {
        apex_assert_hard(parent_->guard_ == -1);
        parent_->addConnection(this);
    }
}

Connection::Connection()
    : parent_(nullptr)
{
}

Connection::~Connection()
{
    if(parent_) {
        detach();
    }
}

void Connection::detach() const
{
    detached_ = true;
    parent_->removeConnection(this);
    parent_ = nullptr;
}

void Connection::disconnect() const
{
    if(parent_) {
        apex_assert_hard(parent_->guard_ == -1);
        if(deleter_) {
            deleter_();
        }
        parent_->removeConnection(this);
        parent_ = nullptr;
    }
}



ScopedConnection::ScopedConnection(const Connection& c)
    : Connection(c)
{
}
ScopedConnection::ScopedConnection(ScopedConnection&& c) noexcept
    : Connection(c)
{
    c.parent_->removeConnection(&c);
    c.parent_ = nullptr;
}
ScopedConnection::ScopedConnection()
{
}

ScopedConnection::~ScopedConnection()
{
    if(parent_) {
        disconnect();
    }
}


void ScopedConnection::operator = (const Connection& c)
{
    apex_assert_hard(c.parent_ != nullptr);
    disconnect();
    deleter_ = c.deleter_;
    parent_ = c.parent_;
    parent_->addConnection(this);
}

void ScopedConnection::operator = (ScopedConnection&& c) noexcept
{
    apex_assert_hard(c.parent_ != nullptr);
    disconnect();
    deleter_ = c.deleter_;
    parent_ = c.parent_;
    c.parent_->removeConnection(&c);
    parent_->addConnection(this);

    c.parent_ = nullptr;
}
