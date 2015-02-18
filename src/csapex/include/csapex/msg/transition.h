#ifndef TRANSITION_H
#define TRANSITION_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <mutex>
#include <vector>

namespace csapex
{

class Transition
{
public:
    Transition(NodeWorker* node);

    NodeWorker* getNode() const;

    void addConnection(ConnectionWeakPtr connection);
    void removeConnection(ConnectionWeakPtr connection);

    void notifyMessageProcessed();
    void fireIfPossible();
    int determineNeededFirings() const;
    void fire();

private:
    NodeWorker* node_;

    std::vector<ConnectionWeakPtr> connections_;
    mutable std::recursive_mutex sync;

    int firings_left_;
};

}

#endif // TRANSITION_H

