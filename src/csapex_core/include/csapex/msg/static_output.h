#ifndef STATIC_OUTPUT_H
#define STATIC_OUTPUT_H

/// COMPONENT
#include <csapex/msg/output.h>

namespace csapex
{
class CSAPEX_CORE_EXPORT StaticOutput : public Output
{
public:
    StaticOutput(const UUID& uuid, ConnectableOwnerWeakPtr owner = ConnectableOwnerWeakPtr());

    void addMessage(TokenPtr message) override;

    bool commitMessages(bool is_activated) override;
    bool hasMessage() override;
    bool hasMarkerMessage() override;
    TokenPtr getToken() const override;

    void disable() override;
    void reset() override;
    void clearBuffer() override;

    TokenPtr getToken();

    TokenPtr getAddedToken() override;

private:
    TokenPtr message_to_send_;

    mutable std::recursive_mutex message_mutex_;
    TokenPtr committed_message_;
};
}  // namespace csapex

#endif  // STATIC_OUTPUT_H
