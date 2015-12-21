/// HEADER
#include <csapex/model/node_modifier.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/factory/message_factory.h>

using namespace csapex;

NodeModifier::NodeModifier(NodeWorker *node_worker, NodeHandle *node_handle)
    : node_worker_(node_worker), node_handle_(node_handle)
{

}

Input* NodeModifier::addInput(ConnectionTypePtr type, const std::string& label, bool dynamic, bool optional)
{
    return node_handle_->addInput(type, label, dynamic, optional);
}

Output* NodeModifier::addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic)
{
    return node_handle_->addOutput(type, label, dynamic);
}


Slot* NodeModifier::addSlot(const std::string& label, std::function<void()> callback)
{
    return node_handle_->addSlot(label, callback, false);
}

Slot* NodeModifier::addActiveSlot(const std::string& label, std::function<void()> callback)
{
    return node_handle_->addSlot(label, callback, true);
}


Trigger* NodeModifier::addTrigger(const std::string& label)
{
    return node_handle_->addTrigger(label);
}




std::vector<Input*> NodeModifier::getMessageInputs() const
{
    // hide parameter inputs from the nodes
    auto vec = node_handle_->getAllInputs();
    std::vector<Input*> result;
    for(auto entry : vec) {
        if(!node_handle_->isParameterInput(entry.get()))  {
            result.push_back(entry.get());
        }
    }
    return result;
}
std::vector<Output*> NodeModifier::getMessageOutputs() const
{
    // hide parameter outputs from the nodes
    auto vec = node_handle_->getAllOutputs();
    std::vector<Output*> result;
    for(auto entry : vec) {
        if(!node_handle_->isParameterOutput(entry.get()))  {
            result.push_back(entry.get());
        }
    }
    return result;
}
std::vector<Slot*> NodeModifier::getSlots() const
{
    auto vec = node_handle_->getSlots();
    std::vector<Slot*> result(vec.size());
    std::size_t i = 0;
    for(auto entry : vec) {
        result[i++] = entry.get();
    }
    return result;
}
std::vector<Trigger*> NodeModifier::getTriggers() const
{
    auto vec = node_handle_->getTriggers();
    std::vector<Trigger*> result(vec.size());
    std::size_t i = 0;
    for(auto entry : vec) {
        result[i++] = entry.get();
    }
    return result;
}


void NodeModifier::removeInput(const UUID &uuid)
{
    node_handle_->removeInput(uuid);
}

void NodeModifier::removeOutput(const UUID &uuid)
{
    node_handle_->removeOutput(uuid);
}

void NodeModifier::removeSlot(const UUID &uuid)
{
    node_handle_->removeSlot(uuid);
}

void NodeModifier::removeTrigger(const UUID &uuid)
{
    node_handle_->removeTrigger(uuid);
}

bool NodeModifier::isSource() const
{
    return node_handle_->isSource();
}
void NodeModifier::setIsSource(bool source)
{
    node_handle_->setIsSource(source);
}

bool NodeModifier::isSink() const
{
    return node_handle_->isSink();
}
void NodeModifier::setIsSink(bool sink)
{
    node_handle_->setIsSink(sink);
}

bool NodeModifier::isError() const
{
    return node_worker_->isError();
}
void NodeModifier::setNoError()
{
    node_worker_->setError(false);
}
void NodeModifier::setError(const std::string &msg)
{
    node_worker_->setError(true, msg, ErrorState::ErrorLevel::ERROR);
}
void NodeModifier::setWarning(const std::string &msg)
{
    node_worker_->setError(true, msg, ErrorState::ErrorLevel::WARNING);
}
