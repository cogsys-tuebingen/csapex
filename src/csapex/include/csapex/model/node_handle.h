#ifndef NODE_HANDLE_H
#define NODE_HANDLE_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/unique.h>

/// SYSTEM
#include <vector>
#include <string>

namespace csapex
{

class NodeHandle : public Unique
{
public:
    NodeHandle(const std::string& type, const UUID &uuid, NodePtr node);
    virtual ~NodeHandle();    

    std::string getType() const;
    NodeWeakPtr getNode() const;

    virtual bool canProcess() const = 0;
    virtual bool isEnabled() const = 0;

    virtual void triggerCheckTransitions() = 0;
    virtual void startProcessingMessages() = 0;

protected:
    std::string node_type_;

    NodePtr node_;

    std::vector<InputPtr> inputs_;
    std::vector<OutputPtr> outputs_;
    std::vector<TriggerPtr> triggers_;
    std::vector<SlotPtr> slots_;

};

}

#endif // NODE_HANDLE_H

