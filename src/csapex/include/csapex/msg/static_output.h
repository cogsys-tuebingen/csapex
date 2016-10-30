#ifndef STATIC_OUTPUT_H
#define STATIC_OUTPUT_H

/// COMPONENT
#include <csapex/msg/output.h>

namespace csapex
{
class CSAPEX_EXPORT StaticOutput : public Output
{
public:
    StaticOutput(const UUID &uuid, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());

    virtual void addMessage(TokenPtr message) override;

    virtual bool commitMessages(bool is_activated) override;
    virtual bool hasMessage() override;
    virtual bool hasMarkerMessage() override;
    virtual void nextMessage() override;
    virtual TokenPtr getToken() const override;

    virtual void disable() override;
    virtual void reset() override;
    virtual void clearBuffer() override;

    TokenPtr getToken();

    TokenPtr getAddedToken() override;

private:
    TokenPtr message_to_send_;

    mutable std::recursive_mutex message_mutex_;
    TokenPtr committed_message_;
};
}

#endif // STATIC_OUTPUT_H

