#ifndef SLOT_H
#define SLOT_H

/// COMPONENT
#include <csapex/signal/signal_fwd.h>
#include <csapex/msg/input.h>
#include <csapex/model/token.h>

/// SYSTEM
#include <mutex>
#include <condition_variable>
#include <deque>

namespace csapex
{

class CSAPEX_EXPORT Slot : public Input
{
    friend class Event;

public:
    Slot(std::function<void()> callback, const UUID &uuid, bool active, bool asynchronous = false, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());
    Slot(std::function<void(const TokenPtr&)> callback, const UUID &uuid, bool active, bool asynchronous = false, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());
    Slot(std::function<void(Slot*,const TokenPtr&)> callback, const UUID &uuid, bool active, bool asynchronous = false, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());

    template <typename TokenType>
    Slot(std::function<void(const std::shared_ptr<TokenType const>&)> callback, const UUID &uuid, bool active)
        : Slot([callback](const TokenPtr& token){ auto t = std::dynamic_pointer_cast<TokenType const>(token->getTokenData()); apex_assert_hard(t); callback(t); }, uuid, active)
    {
    }

    virtual ~Slot();

    virtual void setToken(TokenPtr message) override;

    virtual ConnectorType getConnectorType() const override
    {
        return ConnectorType::SLOT_T;
    }

    bool isActive() const;

    void notifyMessageAvailable(Connection* connection) override;
    void notifyMessageProcessed() override;

    bool canConnectTo(Connectable* other_side, bool move) const override;

    virtual void disable() override;

    void reset();

    void handleEvent();

    void notifyEventHandled();

public:
    csapex::slim_signal::Signal<void(const TokenPtr&)> token_set;
    csapex::slim_signal::Signal<void()> triggered;

protected:
    std::function<void(Slot*,const TokenPtr&)> callback_;
    bool active_;
    bool asynchronous_;

    long guard_;

private:
    std::deque<Connection*> available_connections_;

    std::recursive_mutex available_connections_mutex_;
};

}
#endif // SLOT_H
