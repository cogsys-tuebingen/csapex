#ifndef TRIGGER_H
#define TRIGGER_H

/// COMPONENT
#include <csapex/signal/signal_fwd.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/msg/generic_value_message.hpp>

namespace csapex
{

class CSAPEX_EXPORT Event : public StaticOutput
{
    friend class Graph;

public:
    Event(const UUID &uuid, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());
    ~Event();

    virtual ConnectorType getConnectorType() const override
    {
        return ConnectorType::EVENT;
    }

    /**
     * @brief trigger triggers an event with a "Nothing" token
     */
    void trigger();

    /**
     * @brief triggerWith triggers an event with a specified token
     * @param token
     */
    void triggerWith(TokenPtr token);

    void reset();

public:
    slim_signal::Signal<void()> triggered;

};

}

#endif // TRIGGER_H
