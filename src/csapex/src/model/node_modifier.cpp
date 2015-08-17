/// HEADER
#include <csapex/model/node_modifier.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/msg/message_factory.h>

using namespace csapex;

NodeModifier::NodeModifier(NodeWorker *node)
    : node_worker_(node)
{

}

Input* NodeModifier::addInput(ConnectionTypePtr type, const std::string& label, bool dynamic, bool optional)
{
    return node_worker_->addInput(type, label, dynamic, optional);
}

Output* NodeModifier::addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic)
{
    return node_worker_->addOutput(type, label, dynamic);
}


Slot* NodeModifier::addSlot(const std::string& label, std::function<void()> callback)
{
    return node_worker_->addSlot(label, callback, false);
}

Slot* NodeModifier::addActiveSlot(const std::string& label, std::function<void()> callback)
{
    return node_worker_->addSlot(label, callback, true);
}


Trigger* NodeModifier::addTrigger(const std::string& label)
{
    return node_worker_->addTrigger(label);
}




std::vector<Input*> NodeModifier::getMessageInputs() const
{
    // hide parameter inputs from the nodes
    auto vec = node_worker_->getAllInputs();
    std::vector<Input*> result;
    for(auto entry : vec) {
        if(!node_worker_->isParameterInput(entry.get()))  {
            result.push_back(entry.get());
        }
    }
    return result;
}
std::vector<Output*> NodeModifier::getMessageOutputs() const
{
    // hide parameter outputs from the nodes
    auto vec = node_worker_->getAllOutputs();
    std::vector<Output*> result;
    for(auto entry : vec) {
        if(!node_worker_->isParameterOutput(entry.get()))  {
            result.push_back(entry.get());
        }
    }
    return result;
}
std::vector<Slot*> NodeModifier::getSlots() const
{
    auto vec = node_worker_->getSlots();
    std::vector<Slot*> result(vec.size());
    std::size_t i = 0;
    for(auto entry : vec) {
        result[i++] = entry.get();
    }
    return result;
}
std::vector<Trigger*> NodeModifier::getTriggers() const
{
    auto vec = node_worker_->getTriggers();
    std::vector<Trigger*> result(vec.size());
    std::size_t i = 0;
    for(auto entry : vec) {
        result[i++] = entry.get();
    }
    return result;
}


void NodeModifier::removeInput(const UUID &uuid)
{
    node_worker_->removeInput(uuid);
}

void NodeModifier::removeOutput(const UUID &uuid)
{
    node_worker_->removeOutput(uuid);
}

void NodeModifier::removeSlot(const UUID &uuid)
{
    node_worker_->removeSlot(uuid);
}

void NodeModifier::removeTrigger(const UUID &uuid)
{
    node_worker_->removeTrigger(uuid);
}


void NodeModifier::setTickEnabled(bool tick)
{
    node_worker_->setTickEnabled(tick);
}

void NodeModifier::setTickFrequency(double f)
{
    node_worker_->setTickFrequency(f);
}

bool NodeModifier::isSource() const
{
    return node_worker_->isSource();
}
void NodeModifier::setIsSource(bool source)
{
    node_worker_->setIsSource(source);
}

bool NodeModifier::isSink() const
{
    return node_worker_->isSink();
}
void NodeModifier::setIsSink(bool sink)
{
    node_worker_->setIsSink(sink);
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
