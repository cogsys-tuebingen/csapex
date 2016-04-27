#ifndef SLOT_H
#define SLOT_H

/// COMPONENT
#include <csapex/signal/signal_fwd.h>
#include <csapex/msg/input.h>

/// SYSTEM
#include <mutex>
#include <condition_variable>

namespace csapex
{

class Slot : public Input
{
    friend class Event;

public:
    Slot(std::function<void()> callback, const UUID &uuid, bool active);
    Slot(std::function<void(const TokenConstPtr&)> callback, const UUID &uuid, bool active);

    template <typename TokenType>
    Slot(std::function<void(const std::shared_ptr<TokenType const>&)> callback, const UUID &uuid, bool active)
        : Slot([callback](const TokenConstPtr& token){ auto t = std::dynamic_pointer_cast<TokenType const>(token); apex_assert_hard(t); callback(t); }, uuid, active)
    {
    }

    virtual ~Slot();

    virtual void inputMessage(Token::ConstPtr message) override;

    virtual ConnectorType getConnectorType() const override
    {
        return ConnectorType::SLOT_T;
    }

    bool isActive() const;


    bool canConnectTo(Connectable* other_side, bool move) const override;

    virtual void disable() override;

    void reset();

    void handleEvent();

public:
    csapex::slim_signal::Signal<void()> triggered;

protected:
    std::function<void(const TokenConstPtr&)> callback_;
    bool active_;
};

}
#endif // SLOT_H
