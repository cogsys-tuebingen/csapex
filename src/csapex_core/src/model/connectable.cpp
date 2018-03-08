/// HEADER
#include <csapex/model/connectable.h>

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/msg/message.h>
#include <csapex/msg/any_message.h>
#include <csapex/utility/debug.h>
#include <csapex/model/connectable_owner.h>

/// SYSTEM
#include <iostream>
#include <sstream>
#include <typeinfo>

using namespace csapex;


//bool Connectable::allow_processing = true;


Connectable::Connectable(const UUID& uuid, ConnectableOwnerWeakPtr owner)
    : Connector(uuid, owner),
      count_(0), seq_no_(-1),
      virtual_(false),
      parameter_(false),
      variadic_(false),
      graph_port_(false),
      essential_(false),
      enabled_(true)
{
    init();
}

bool Connectable::isVirtual() const
{
    return virtual_;
}

void Connectable::setVirtual(bool _virtual)
{
    virtual_ = _virtual;
}


bool Connectable::isVariadic() const
{
    return variadic_;
}

void Connectable::setVariadic(bool variadic)
{
    variadic_ = variadic;
}

bool Connectable::isParameter() const
{
    return parameter_;
}

void Connectable::setParameter(bool parameter)
{
    parameter_ = parameter;
}


bool Connectable::isGraphPort() const
{
    return graph_port_;
}

void Connectable::setGraphPort(bool graph_port)
{
    graph_port_ = graph_port;
}

bool Connectable::isEssential() const
{
    return essential_;
}

void Connectable::setEssential(bool essential)
{
    if(essential != essential_) {
        essential_ = essential;
        essential_changed(essential_);
    }
}

void Connectable::notifyMessageProcessed()
{
    message_processed(shared_from_this());

    APEX_DEBUG_CERR <<"connectable " << getUUID() << " notified" << std::endl;

    for(const ConnectionPtr& c : connections_) {
        c->setTokenProcessed();
    }
}

void Connectable::reset()
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    message_processed(shared_from_this());
}

void Connectable::stop()
{
    //notifyMessageProcessed();
}

void Connectable::init()
{
    setType(connection_types::makeEmpty<connection_types::AnyMessage>());
}


Connectable::~Connectable()
{
    for(const ConnectionPtr& c : connections_) {
        c->detach(this);
    }
}

void Connectable::disable()
{
    if(enabled_) {
        enabled_ = false;
        enabled_changed((bool) enabled_);
    }
}

void Connectable::enable()
{
    if(!enabled_) {
        enabled_ = true;
        enabled_changed((bool) enabled_);
    }
}

void Connectable::setEnabled(bool enabled)
{
    if(enabled) {
        enable();
    } else {
        disable();
    }
}

bool Connectable::isEnabled() const
{
    return enabled_;
}

std::string Connectable::getLabel() const
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    return label_;
}

void Connectable::setLabel(const std::string &label)
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    if(label != label_) {
        label_ = label;
        labelChanged(label_);
    }
}

void Connectable::setType(TokenData::ConstPtr type)
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    bool compatible = type_ && type && type_->canConnectTo(type.get()) && type->canConnectTo(type_.get());

    bool is_any = std::dynamic_pointer_cast<connection_types::AnyMessage const>(type_) != nullptr;
    bool will_be_any = std::dynamic_pointer_cast<connection_types::AnyMessage const>(type) != nullptr;

    if(!compatible || (is_any != will_be_any)) {
        type_ = type;
        lock.unlock();

        typeChanged(type);
    }
}

TokenData::ConstPtr Connectable::getType() const
{
    return type_;
}


int Connectable::getCount() const
{
    return count_;
}

int Connectable::sequenceNumber() const
{
    return seq_no_;
}

void Connectable::setSequenceNumber(int seq_no)
{
    seq_no_ = seq_no;
}

void Connectable::removeConnection(Connectable* other_side)
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    for(std::vector<ConnectionPtr>::iterator i = connections_.begin(); i != connections_.end();) {
        ConnectionPtr c = *i;
        Connector* f = c->source().get();
        Connector* t = c->target().get();
        if((t == other_side) || (f == other_side)) {
            apex_assert_hard((this == t) ^ (this == f));

            lock.unlock();

            fadeConnection(c);
            other_side->fadeConnection(c);

            return;

        } else {
            ++i;
        }
    }
}


void Connectable::addConnection(ConnectionPtr connection)
{
    connections_.push_back(connection);
    connection->sink_enabled_changed.connect(connectionEnabled);

    connection_added(connection);

    connection_added_to(shared_from_this());
}

void Connectable::fadeConnection(ConnectionPtr connection)
{
    for(auto it = connections_.begin(); it != connections_.end(); ) {
        if(*it == connection) {
            it = connections_.erase(it);
        } else {
            ++it;
        }
    }


    connection_removed_to(shared_from_this());

    connection_faded(connection);
}

int Connectable::countConnections() const
{
    return connections_.size();
}
int Connectable::maxConnectionCount() const
{
    return -1;
}

std::vector<ConnectionPtr> Connectable::getConnections() const
{
    return connections_;
}

bool Connectable::hasActiveConnection() const
{
    for(const ConnectionPtr& c : connections_) {
        if(c->isActive()) {
            return true;
        }
    }

    return false;
}

bool Connectable::hasEnabledConnection() const
{
    for(const ConnectionPtr& c : connections_) {
        if(c->isEnabled()) {
            return true;
        }
    }

    return false;
}

bool Connectable::isConnected() const
{
    return !connections_.empty();
}

bool Connectable::isConnectedTo(const UUID& other) const
{
    for(const ConnectionPtr& c : connections_) {
        ConnectorPtr other_port = isInput() ? c->source() : c->target();
        if(other_port->getUUID() == other) {
            return true;
        }
    }
    return false;
}

bool Connectable::isActivelyConnectedTo(const UUID& other) const
{
    for(const ConnectionPtr& c : connections_) {
        ConnectorPtr other_port = isInput() ? c->source() : c->target();
        if(other_port->getUUID() == other) {
            return c->isActive();
        }
    }
    return false;
}


std::vector<UUID> Connectable::getConnectedPorts() const
{
    std::vector<UUID> res;
    res.reserve(connections_.size());
    for(const ConnectionPtr& c : connections_) {
        ConnectorPtr other_port = isInput() ? c->source() : c->target();
        UUID uuid = other_port->getUUID();
        if(!uuid.empty()) {
            res.push_back(uuid);
        }
    }

    return res;
}


std::string Connectable::makeStatusString() const
{
    std::stringstream status_stream;
    status_stream << "UUID: " << getUUID();
    status_stream << ", Type: " << getType()->descriptiveName();
    status_stream << ", Connections: " << getConnections().size();
    status_stream << ", Messages: " << getCount();
    status_stream << ", Enabled: " << isEnabled();
    status_stream << ", #: " << sequenceNumber();

    addStatusInformation(status_stream);

    return status_stream.str();
}

ConnectorDescription Connectable::getDescription() const
{
    auto owner = getOwner();
    if(!owner) {
        return {};
    }
    ConnectorDescription res(owner->getUUID().getAbsoluteUUID(),
                             getUUID(),
                             getConnectorType(),
                             getType(),
                             getLabel());
    res.setParameter(isParameter())
            .setOptional(isOptional())
            .setVariadic(isVariadic());

    for(const ConnectionPtr& c : getConnections()) {
        if(isOutput()) {
            res.targets.emplace_back(c->target()->getUUID().getAbsoluteUUID(), c->isActive());
        } else {
            res.targets.emplace_back(c->source()->getUUID().getAbsoluteUUID(), c->isActive());
        }
    }
    return res;
}
