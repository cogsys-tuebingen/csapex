/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/q_signal_relay.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;

Node::Node(const UUID &uuid)
    : Unique(uuid),
      modifier_(new NodeModifier(this)),
      ainfo(std::cout, uuid.getFullName()), awarn(std::cout, uuid.getFullName()), aerr(std::cerr, uuid.getFullName()), alog(std::clog, uuid.getFullName()),
      settings_(NULL), worker_(NULL),
      node_state_(new NodeState(this)), dispatcher_(NULL)
{
}

Node::~Node()
{
    while(!inputs_.empty()) {
        removeInput(*inputs_.begin());
    }
    while(!outputs_.empty()) {
        removeOutput(*outputs_.begin());
    }
    while(!managed_inputs_.empty()) {
        removeInput(*managed_inputs_.begin());
    }
    while(!managed_outputs_.empty()) {
        removeOutput(*managed_outputs_.begin());
    }

    Q_FOREACH(QObject* cb, callbacks) {
        qt_helper::Call* call = dynamic_cast<qt_helper::Call*>(cb);
        if(call) {
            call->disconnect();
        }
        cb->deleteLater();
    }
    callbacks.clear();

    delete worker_;
    delete modifier_;
}

void Node::setUUID(const UUID &uuid)
{
    Unique::setUUID(uuid);

    std::string p = uuid.getFullName();
    ainfo.setPrefix(p);
    awarn.setPrefix(p);
    aerr.setPrefix(p);
    alog.setPrefix(p);
}

void Node::doSetup()
{
    setupParameters();
    updateParameters();

    try {
        setup();
    } catch(std::runtime_error& e) {
        aerr << "setup failed: " << e.what() << std::endl;
    }
}

void Node::setType(const std::string &type)
{
    type_ = type;
}

std::string Node::getType() const
{
    return type_;
}

ConnectorIn* Node::getParameterInput(const std::string &name) const
{
    std::map<std::string, ConnectorIn*>::const_iterator it = param_2_input_.find(name);
    if(it == param_2_input_.end()) {
        return NULL;
    } else {
        return it->second;
    }
}

ConnectorOut* Node::getParameterOutput(const std::string &name) const
{
    std::map<std::string, ConnectorOut*>::const_iterator it = param_2_output_.find(name);
    if(it == param_2_output_.end()) {
        return NULL;
    } else {
        return it->second;
    }
}

bool Node::canBeDisabled() const
{
    return true;
}

bool Node::isEnabled()
{
    return node_state_->isEnabled();
}
void Node::messageArrived(ConnectorIn *)
{

}
void Node::setupParameters()
{

}

void Node::process()
{
}

void Node::checkIO()
{
    if(isEnabled()) {
        enableInput(canReceive());
        enableOutput(canReceive());
    } else {
        enableInput(false);
        enableOutput(false);
    }
}

Settings& Node::getSettings()
{
    return *settings_;
}

NodeState::Ptr Node::getNodeStateCopy() const
{
    apex_assert_hard(node_state_);

    NodeState::Ptr memento(new NodeState(this));
    *memento = *node_state_;

    memento->setChildState(getChildState());

    return memento;
}

NodeState::Ptr Node::getNodeState()
{
    apex_assert_hard(node_state_);

    return node_state_;
}

void Node::setNodeState(NodeState::Ptr memento)
{
    boost::shared_ptr<NodeState> m = boost::dynamic_pointer_cast<NodeState> (memento);
    apex_assert_hard(m.get());

    UUID old_uuid = getUUID();
    std::string old_label = node_state_->getLabel();

    *node_state_ = *m;

    if(getUUID().empty()) {
        setUUID(old_uuid);
    }
    if(node_state_->getLabel().empty()) {
        if(old_label.empty()) {
            node_state_->setLabel(getUUID().getShortName());
        } else {
            node_state_->setLabel(old_label);
        }
    }

    node_state_->setParent(this);
    if(m->getChildState()) {
        setState(m->getChildState());
    }

    Q_EMIT stateChanged();
}

template <typename T>
void Node::updateParameter(param::Parameter *p)
{
    {
        ConnectorIn* cin;
        cin = new ConnectorIn(*settings_, UUID::make_sub(getUUID(), p->name() + "_in"));

        cin->setType(connection_types::GenericValueMessage<T>::make());

        cin->enable();
        /// TODO: make synchronized!!!!!
        cin->setAsync(true);

        boost::function<typename connection_types::GenericValueMessage<T>::Ptr()> getmsgptr = boost::bind(&ConnectorIn::getMessage<connection_types::GenericValueMessage<T> >, cin, (void*) 0);
        boost::function<connection_types::GenericValueMessage<T>*()> getmsg = boost::bind(&connection_types::GenericValueMessage<T>::Ptr::get, boost::bind(getmsgptr));
        boost::function<T()> read = boost::bind(&connection_types::GenericValueMessage<T>::getValue, boost::bind(getmsg));
        boost::function<void()> set_params_fn = boost::bind(&param::Parameter::set<T>, p, boost::bind(read));
        qt_helper::Call* set_param = new qt_helper::Call(set_params_fn);
        callbacks.push_back(set_param);
        QObject::connect(cin, SIGNAL(messageArrived(Connectable*)), set_param, SLOT(call()));

        manageInput(cin);
        param_2_input_[p->name()] = cin;
    }
    {
        ConnectorOut* cout;
        cout = new ConnectorOut(*settings_, UUID::make_sub(getUUID(), p->name() + "_out"));

        cout->setType(connection_types::GenericValueMessage<T>::make());

        cout->enable();
        /// TODO: make synchronized!!!!!
        cout->setAsync(true);

        boost::function<void(T)> publish = boost::bind(&ConnectorOut::publishIntegral<T>, cout, _1, "/");
        boost::function<T()> read = boost::bind(&param::Parameter::as<T>, p);
        connections.push_back(parameter_changed(*p).connect(boost::bind(publish, boost::bind(read))));

        manageOutput(cout);
        param_2_output_[p->name()] = cout;
    }
}

void Node::updateParameters()
{
    apex_assert_hard(!getUUID().empty());

    for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = parameter_state_.params.begin(); it != parameter_state_.params.end(); ++it ) {
        param::Parameter* p = it->second.get();

        if(p->is<int>()) {
            updateParameter<int>(p);
        } else if(p->is<double>()) {
            updateParameter<double>(p);
        } else if(p->is<std::string>()) {
            updateParameter<std::string>(p);
        } else if(p->is<bool>()) {
            updateParameter<bool>(p);
        } else if(p->is<std::pair<int, int> >()) {
            updateParameter<std::pair<int, int> >(p);
        } else if(p->is<std::pair<double, double> >()) {
            updateParameter<std::pair<double, double> >(p);
        }
        // else: do nothing and ignore the parameter
    }
}

Memento::Ptr Node::getChildState() const
{
    return parameter_state_.clone();
}

void Node::setState(Memento::Ptr memento)
{
    boost::shared_ptr<GenericState> m = boost::dynamic_pointer_cast<GenericState> (memento);
    apex_assert_hard(m.get());

    parameter_state_.setFrom(*m);

    Q_EMIT modelChanged();
}

void Node::enable(bool e)
{
    if(e) {
        enable();
    } else {
        disable();
    }
}

void Node::enable()
{
    node_state_->setEnabled(true);
    checkIO();

    Q_EMIT enabled(true);
}

void Node::disable(bool d)
{
    enable(!d);
}


void Node::disable()
{
    node_state_->setEnabled(false);
    setError(false);
    checkIO();

    Q_EMIT enabled(false);
}

bool Node::canReceive()
{
    bool can_receive = true;
    Q_FOREACH(ConnectorIn* i, inputs_) {
        if(!i->isConnected() && !i->isOptional()) {
            can_receive = false;
        } else if(i->isConnected() && !i->getSource()->isEnabled()) {
            can_receive = false;
        }
    }

    return can_receive;
}

void Node::enableIO(bool enable)
{
    enableInput(canReceive() && enable);
    enableOutput(enable);
}

void Node::enableInput (bool enable)
{
    Q_FOREACH(ConnectorIn* i, inputs_) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}


void Node::enableOutput (bool enable)
{
    Q_FOREACH(ConnectorOut* o, outputs_) {
        if(enable) {
            o->enable();
        } else {
            o->disable();
        }
    }
}

void Node::setIOError(bool error)
{
    Q_FOREACH(ConnectorIn* i, inputs_) {
        i->setErrorSilent(error);
    }
    Q_FOREACH(ConnectorOut* i, outputs_) {
        i->setErrorSilent(error);
    }
    enableIO(!error);
}

void Node::setMinimized(bool min)
{
    node_state_->setMinimized(min);
}

void Node::triggerModelChanged()
{
    Q_EMIT modelChanged();
}

void Node::connectorChanged()
{

}

void Node::tick()
{
}

void Node::updateModel()
{
}

void Node::setSettings(Settings *settings)
{
    settings_ = settings;
}

void Node::setNodeWorker(NodeWorker *nw)
{
    worker_ = nw;
}

NodeWorker* Node::getNodeWorker() const
{
    return worker_;
}


void Node::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    Q_EMIT nodeError(error,msg,level);

    if(node_state_->isEnabled() && error && level == EL_ERROR) {
        setIOError(true);
    } else {
        setIOError(false);
    }
}



ConnectorIn* Node::addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async)
{
    int id = inputs_.size();
    ConnectorIn* c = new ConnectorIn(*settings_, this, id);
    c->setLabel(label);
    c->setOptional(optional);
    c->setAsync(async);
    c->setType(type);

    registerInput(c);

    return c;
}

ConnectorOut* Node::addOutput(ConnectionTypePtr type, const std::string& label)
{
    int id = outputs_.size();
    ConnectorOut* c = new ConnectorOut(*settings_, this, id);
    c->setLabel(label);
    c->setType(type);

    registerOutput(c);

    return c;
}

void Node::addInput(ConnectorIn* in)
{
    registerInput(in);
}

void Node::addOutput(ConnectorOut* out)
{
    registerOutput(out);
}


void Node::manageInput(ConnectorIn* in)
{
    managed_inputs_.push_back(in);
    connectConnector(in);
    in->moveToThread(thread());
}

void Node::manageOutput(ConnectorOut* out)
{
    managed_outputs_.push_back(out);
    connectConnector(out);
    out->moveToThread(thread());
}

int Node::countInputs() const
{
    return inputs_.size();
}

int Node::countOutputs() const
{
    return outputs_.size();
}

int Node::countManagedInputs() const
{
    return managed_inputs_.size();
}

int Node::countManagedOutputs() const
{
    return managed_outputs_.size();
}

ConnectorIn* Node::getInput(const unsigned int index) const
{
    apex_assert_hard(index < inputs_.size());
    return inputs_[index];
}

ConnectorOut* Node::getOutput(const unsigned int index) const
{
    apex_assert_hard(index < outputs_.size());
    return outputs_[index];
}


ConnectorIn* Node::getManagedInput(const unsigned int index) const
{
    apex_assert_hard(index < managed_inputs_.size());
    return managed_inputs_[index];
}

ConnectorOut* Node::getManagedOutput(const unsigned int index) const
{
    apex_assert_hard(index < managed_outputs_.size());
    return managed_outputs_[index];
}

ConnectorIn* Node::getInput(const UUID& uuid) const
{
    BOOST_FOREACH(ConnectorIn* in, inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }
    BOOST_FOREACH(ConnectorIn* in, managed_inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }

    return NULL;
}

ConnectorOut* Node::getOutput(const UUID& uuid) const
{
    BOOST_FOREACH(ConnectorOut* out, outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }
    BOOST_FOREACH(ConnectorOut* out, managed_outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }

    return NULL;
}

Connectable* Node::getConnector(const UUID &uuid) const
{
    Connectable* result = getInput(uuid);

    if(result == NULL) {
        result = getOutput(uuid);
    }

    return result;
}

std::vector<ConnectorIn*> Node::getInputs() const
{
    std::vector<ConnectorIn*> result;
    result = inputs_;
    result.insert(result.end(), managed_inputs_.begin(), managed_inputs_.end());
    return result;
}

std::vector<ConnectorOut*> Node::getOutputs() const
{
    std::vector<ConnectorOut*> result;
    result = outputs_;
    result.insert(result.end(), managed_outputs_.begin(), managed_outputs_.end());
    return result;
}

void Node::removeInput(ConnectorIn *in)
{

    if(worker_) {
        worker_->removeInput(in);
    }
    std::vector<ConnectorIn*>::iterator it;
    it = std::find(inputs_.begin(), inputs_.end(), in);

    if(it != inputs_.end()) {
        inputs_.erase(it);
    } else {
        it = std::find(managed_inputs_.begin(), managed_inputs_.end(), in);
        if(it != managed_inputs_.end()) {
            managed_inputs_.erase(it);
        } else {
            std::cerr << "ERROR: cannot remove input " << in->getUUID().getFullName() << std::endl;
        }
    }

    in->deleteLater();

    disconnectConnector(in);
    Q_EMIT connectorRemoved(in);
}

void Node::removeOutput(ConnectorOut *out)
{
    std::vector<ConnectorOut*>::iterator it;
    it = std::find(outputs_.begin(), outputs_.end(), out);

    if(it != outputs_.end()) {
        outputs_.erase(it);
    } else {
        it = std::find(managed_outputs_.begin(), managed_outputs_.end(), out);
        if(it != managed_outputs_.end()) {
            managed_outputs_.erase(it);
        } else {
            std::cerr << "ERROR: cannot remove output " << out->getUUID().getFullName() << std::endl;
        }
    }

    out->deleteLater();

    disconnectConnector(out);
    Q_EMIT connectorRemoved(out);
}


Command::Ptr Node::removeAllConnectionsCmd()
{
    command::Meta::Ptr cmd(new command::Meta("Remove All Connections"));

    BOOST_FOREACH(ConnectorIn* i, getInputs()) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }
    BOOST_FOREACH(ConnectorOut* i, getOutputs()) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }

    return cmd;
}

void Node::registerInput(ConnectorIn* in)
{
    inputs_.push_back(in);
    in->setCommandDispatcher(dispatcher_);

    if(!worker_) {
        return;
    }
    in->moveToThread(thread());



    worker_->addInput(in);
    connectConnector(in);
    QObject::connect(in, SIGNAL(messageArrived(Connectable*)), worker_, SLOT(forwardMessage(Connectable*)));

    Q_EMIT connectorCreated(in);
}

void Node::registerOutput(ConnectorOut* out)
{
    out->moveToThread(thread());

    outputs_.push_back(out);

    out->setCommandDispatcher(dispatcher_);

    connectConnector(out);

    Q_EMIT connectorCreated(out);
}

int Node::nextInputId()
{
    return inputs_.size();
}

int Node::nextOutputId()
{
    return outputs_.size();
}

CommandDispatcher* Node::getCommandDispatcher() const
{
    return dispatcher_;
}

void Node::setCommandDispatcher(CommandDispatcher *d)
{
    dispatcher_ = d;
}

void Node::useTimer(Timer *timer)
{
    Timable::useTimer(timer);

    Q_FOREACH(ConnectorOut* i, outputs_) {
        i->useTimer(timer);
    }
}

void Node::pause(bool pause)
{
    worker_->pause(pause);
}

void Node::clearBlock()
{
    Q_FOREACH(ConnectorIn* i, inputs_) {
        if(i->isBlocked()) {
            i->free();
            worker_->clearInput(i);
        }
    }
    Q_FOREACH(ConnectorIn* i, inputs_) {
        i->setSequenceNumber(0);
    }
    Q_FOREACH(ConnectorOut* o, outputs_) {
        o->setSequenceNumber(0);
    }
}

void Node::stop()
{
    if(worker_) {
        worker_->stop();
    }

    Q_FOREACH(ConnectorIn* i, inputs_) {
        i->free();
    }
    Q_FOREACH(ConnectorOut* i, outputs_) {
        i->stop();
    }
    Q_FOREACH(ConnectorIn* i, inputs_) {
        i->stop();
    }

    Q_FOREACH(ConnectorIn* i, inputs_) {
        disconnectConnector(i);
    }
    Q_FOREACH(ConnectorOut* i, outputs_) {
        disconnectConnector(i);
    }

    QObject::disconnect(worker_);
    QObject::disconnect(this);
}

void Node::connectConnector(Connectable *c)
{
    QObject::connect(c, SIGNAL(connectionInProgress(Connectable*,Connectable*)), this, SIGNAL(connectionInProgress(Connectable*,Connectable*)));
    QObject::connect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    QObject::connect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
    QObject::connect(c, SIGNAL(connectionDone()), this, SLOT(checkIO()));
    QObject::connect(c, SIGNAL(connectionEnabled(bool)), this, SLOT(checkIO()));
    QObject::connect(c, SIGNAL(connectionRemoved()), this, SLOT(checkIO()));
}


void Node::disconnectConnector(Connectable */*c*/)
{
    //    QObject::disconnect(c, SIGNAL(connectionInProgress(Connectable*,Connectable*)), this, SIGNAL(connectionInProgress(Connectable*,Connectable*)));
    //    QObject::disconnect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    //    QObject::disconnect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
}
