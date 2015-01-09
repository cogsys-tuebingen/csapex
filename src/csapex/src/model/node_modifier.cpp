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

Input* NodeModifier::addInput(ConnectionTypePtr type, const std::string& label, bool optional)
{
    return node_worker_->addInput(type, label, optional);
}

Output* NodeModifier::addOutput(ConnectionTypePtr type, const std::string& label)
{
    return node_worker_->addOutput(type, label);
}


Slot* NodeModifier::addSlot(const std::string& label, boost::function<void()> callback)
{
    return node_worker_->addSlot(label, callback, false);
}

Slot* NodeModifier::addActiveSlot(const std::string& label, boost::function<void()> callback)
{
    return node_worker_->addSlot(label, callback, true);
}


Trigger* NodeModifier::addTrigger(const std::string& label)
{
    return node_worker_->addTrigger(label);
}




std::vector<Input*> NodeModifier::getMessageInputs() const
{
    return node_worker_->getMessageInputs();
}
std::vector<Output*> NodeModifier::getMessageOutputs() const
{
    return node_worker_->getMessageOutputs();
}
std::vector<Slot*> NodeModifier::getSlots() const
{
    return node_worker_->getSlots();
}
std::vector<Trigger*> NodeModifier::getTriggers() const
{
    return node_worker_->getTriggers();
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
