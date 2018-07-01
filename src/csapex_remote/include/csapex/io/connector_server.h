#ifndef CONNECTOR_SERVER_H
#define CONNECTOR_SERVER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/io/remote_io_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{
class ConnectorServer : public Observer
{
public:
    ConnectorServer(SessionPtr session);
    ~ConnectorServer();

    void startObserving(const ConnectablePtr& connector);
    void stopObserving(const ConnectablePtr& connector);

private:
    SessionPtr session_;

    std::unordered_map<AUUID, io::ChannelPtr, AUUID::Hasher> channels_;
};
}  // namespace csapex

#endif  // CONNECTOR_SERVER_H
