#ifndef SLOT_H
#define SLOT_H

/// COMPONENT
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/connectable.h>

/// SYSTEM
#include <mutex>
#include <condition_variable>

namespace csapex
{

class Slot : public Connectable
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

    virtual void trigger(Event *source, TokenConstPtr token);

    virtual bool canInput() const override {
        return true;
    }
    virtual bool isInput() const override {
        return true;
    }
    virtual ConnectorType getConnectorType() const override
    {
        return ConnectorType::SLOT_T;
    }

    bool isActive() const;

    virtual bool canConnectTo(Connectable* other_side, bool move) const override;


    virtual bool targetsCanBeMovedTo(Connectable* other_side) const override;
    virtual bool isConnected() const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    std::vector<Event*> getSources() const;

    virtual void enable() override;
    virtual void disable() override;

    virtual void notifyMessageProcessed() override;

    void reset();

    void handleEvent();

public:
    csapex::slim_signal::Signal<void(Event*)> triggered;
    csapex::slim_signal::Signal<void()> connected;

protected:
    bool acknowledgeConnection(Connectable* other_side);

    virtual bool isConnectionPossible(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;
    virtual void removeAllConnectionsNotUndoable() override;

protected:
    std::vector<Event*> sources_;

    std::function<void(const TokenConstPtr&)> callback_;
    bool active_;

    TokenConstPtr current_token_;
};

}
#endif // SLOT_H
