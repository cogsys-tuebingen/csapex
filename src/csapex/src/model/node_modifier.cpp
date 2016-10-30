/// HEADER
#include <csapex/model/node_modifier.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/factory/message_factory.h>
#include <csapex/msg/any_message.h>

using namespace csapex;

NodeModifier::NodeModifier()
    : node_worker_(nullptr)
{
}
NodeModifier::~NodeModifier()
{

}

void NodeModifier::setNodeWorker(NodeWorker *worker)
{
    node_worker_ = worker;
    node_worker_->destroyed.connect([this]() {
        node_worker_ = nullptr;
    });
}


Slot* NodeModifier::addSlot(const std::string& label, std::function<void()> callback, bool active, bool asynchronous)
{
    return addSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), label, [callback](const TokenPtr&) {callback();}, active, asynchronous);
}
Slot* NodeModifier::addActiveSlot(const std::string& label, std::function<void()> callback, bool asynchronous)
{
    return addSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), label, [callback](const TokenPtr&) {callback();}, true, asynchronous);
}


Event* NodeModifier::addEvent(const std::string &label)
{
    return addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), label);
}




std::vector<Input*> NodeModifier::getMessageInputs() const
{
    // hide parameter inputs from the nodes
    auto vec = getExternalInputs();
    std::vector<Input*> result;
    for(auto entry : vec) {
        if(!isParameterInput(entry.get()))  {
            result.push_back(entry.get());
        }
    }
    return result;
}
std::vector<Output*> NodeModifier::getMessageOutputs() const
{
    // hide parameter outputs from the nodes
    auto vec = getExternalOutputs();
    std::vector<Output*> result;
    for(auto entry : vec) {
        if(!isParameterOutput(entry.get()))  {
            result.push_back(entry.get());
        }
    }
    return result;
}
std::vector<Slot*> NodeModifier::getSlots() const
{
    auto vec = getExternalSlots();
    std::vector<Slot*> result(vec.size());
    std::size_t i = 0;
    for(auto entry : vec) {
        result[i++] = entry.get();
    }
    return result;
}
std::vector<Event*> NodeModifier::getEvents() const
{
    auto vec = getExternalEvents();
    std::vector<Event*> result(vec.size());
    std::size_t i = 0;
    for(auto entry : vec) {
        result[i++] = entry.get();
    }
    return result;
}

bool NodeModifier::isProcessingEnabled() const
{
    if(!node_worker_) {
        return false;
    }
    return node_worker_->isProcessingEnabled();
}
void NodeModifier::setProcessingEnabled(bool enabled)
{
    if(node_worker_) {
        node_worker_->setProcessingEnabled(enabled);
    }
}


bool NodeModifier::isError() const
{
    if(!node_worker_) {
        return false;
    }
    return node_worker_->isError();
}
void NodeModifier::setNoError()
{
    if(node_worker_) {
        node_worker_->setError(false);
    }
}
void NodeModifier::setError(const std::string &msg)
{
    if(node_worker_) {
        node_worker_->setError(true, msg, ErrorState::ErrorLevel::ERROR);
    }
}
void NodeModifier::setWarning(const std::string &msg)
{
    if(node_worker_) {
        node_worker_->setError(true, msg, ErrorState::ErrorLevel::WARNING);
    }
}

NodeWorker* NodeModifier::getNodeWorker() const
{
    return node_worker_;
}
