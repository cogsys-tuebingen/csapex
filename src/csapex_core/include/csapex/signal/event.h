#ifndef TRIGGER_H
#define TRIGGER_H

/// COMPONENT
#include <csapex/signal/signal_fwd.h>
#include <csapex/msg/static_output.h>

namespace csapex
{

class CSAPEX_CORE_EXPORT Event : public StaticOutput
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


    virtual bool isSynchronous() const override;

    void reset();


public:
    slim_signal::Signal<void()> triggered;

};

}

#endif // TRIGGER_H
