#ifndef NODE_SERVER_H
#define NODE_SERVER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/io/io_fwd.h>
#include <csapex/model/observer.h>

namespace csapex
{
class NodeServer : public Observer
{
public:
    NodeServer(SessionPtr session);
    ~NodeServer();

    void startObserving(const NodeFacadeLocalPtr &graph);
    void stopObserving(const NodeFacadeLocalPtr& graph);

private:
    SessionPtr session_;
};
}

#endif // NODE_SERVER_H
