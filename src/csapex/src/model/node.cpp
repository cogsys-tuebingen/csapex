/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
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
      modifier_(new NodeModifier(this)), settings_(NULL), dispatcher_(NULL),
      ainfo(std::cout, uuid.getFullName()), awarn(std::cout, uuid.getFullName()), aerr(std::cerr, uuid.getFullName()), alog(std::clog, uuid.getFullName()),
      worker_(NULL),
      node_state_(new NodeState(this))
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

Input* Node::getParameterInput(const std::string &name) const
{
    std::map<std::string, Input*>::const_iterator it = param_2_input_.find(name);
    if(it == param_2_input_.end()) {
        return NULL;
    } else {
        return it->second;
    }
}

Output* Node::getParameterOutput(const std::string &name) const
{
    std::map<std::string, Output*>::const_iterator it = param_2_output_.find(name);
    if(it == param_2_output_.end()) {
        return NULL;
    } else {
        return it->second;
    }
}

void Node::messageArrived(Input *)
{

}
void Node::setupParameters()
{

}

void Node::process()
{
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
        Input* cin;
        cin = new Input(*settings_, UUID::make_sub(getUUID(), p->name() + "_in"));

        cin->setType(connection_types::GenericValueMessage<T>::make());

        cin->enable();
        /// TODO: make synchronized!!!!!
        cin->setAsync(true);

        boost::function<typename connection_types::GenericValueMessage<T>::Ptr()> getmsgptr = boost::bind(&Input::getMessage<connection_types::GenericValueMessage<T> >, cin, (void*) 0);
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
        Output* cout;
        cout = new Output(*settings_, UUID::make_sub(getUUID(), p->name() + "_out"));

        cout->setType(connection_types::GenericValueMessage<T>::make());

        cout->enable();
        /// TODO: make synchronized!!!!!
        cout->setAsync(true);

        boost::function<void(T)> publish = boost::bind(&Output::publishIntegral<T>, cout, _1, "/");
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

    triggerModelChanged();
}

bool Node::canReceive()
{
    bool can_receive = true;
    Q_FOREACH(Input* i, inputs_) {
        if(!i->isConnected() && !i->isOptional()) {
            can_receive = false;
        } else if(i->isConnected() && !i->getSource()->isEnabled()) {
            can_receive = false;
        }
    }

    return can_receive;
}


void Node::triggerModelChanged()
{
    Q_EMIT modelChanged();
}

void Node::tick()
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
        worker_->setIOError(true);
    } else {
        worker_->setIOError(false);
    }
}



Input* Node::addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async)
{
    int id = inputs_.size();
    Input* c = new Input(*settings_, this, id);
    c->setLabel(label);
    c->setOptional(optional);
    c->setAsync(async);
    c->setType(type);

    registerInput(c);

    return c;
}

Output* Node::addOutput(ConnectionTypePtr type, const std::string& label)
{
    int id = outputs_.size();
    Output* c = new Output(*settings_, this, id);
    c->setLabel(label);
    c->setType(type);

    registerOutput(c);

    return c;
}

void Node::manageInput(Input* in)
{
    managed_inputs_.push_back(in);
    connectConnector(in);
    in->moveToThread(thread());
}

void Node::manageOutput(Output* out)
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

Input* Node::getInput(const unsigned int index) const
{
    apex_assert_hard(index < inputs_.size());
    return inputs_[index];
}

Output* Node::getOutput(const unsigned int index) const
{
    apex_assert_hard(index < outputs_.size());
    return outputs_[index];
}


Input* Node::getManagedInput(const unsigned int index) const
{
    apex_assert_hard(index < managed_inputs_.size());
    return managed_inputs_[index];
}

Output* Node::getManagedOutput(const unsigned int index) const
{
    apex_assert_hard(index < managed_outputs_.size());
    return managed_outputs_[index];
}

Input* Node::getInput(const UUID& uuid) const
{
    BOOST_FOREACH(Input* in, inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }
    BOOST_FOREACH(Input* in, managed_inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }

    return NULL;
}

Output* Node::getOutput(const UUID& uuid) const
{
    BOOST_FOREACH(Output* out, outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }
    BOOST_FOREACH(Output* out, managed_outputs_) {
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

std::vector<Input*> Node::getInputs() const
{
    std::vector<Input*> result;
    result = inputs_;
    result.insert(result.end(), managed_inputs_.begin(), managed_inputs_.end());
    return result;
}

std::vector<Output*> Node::getOutputs() const
{
    std::vector<Output*> result;
    result = outputs_;
    result.insert(result.end(), managed_outputs_.begin(), managed_outputs_.end());
    return result;
}

void Node::removeInput(Input *in)
{

    if(worker_) {
        worker_->removeInput(in);
    }
    std::vector<Input*>::iterator it;
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

void Node::removeOutput(Output *out)
{
    std::vector<Output*>::iterator it;
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

void Node::registerInput(Input* in)
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

void Node::registerOutput(Output* out)
{
    out->moveToThread(thread());

    outputs_.push_back(out);

    out->setCommandDispatcher(dispatcher_);

    connectConnector(out);

    Q_EMIT connectorCreated(out);
}

void Node::setCommandDispatcher(CommandDispatcher *d)
{
    dispatcher_ = d;
}

void Node::stop()
{
    if(worker_) {
        worker_->stop();
    }

    Q_FOREACH(Input* i, inputs_) {
        i->free();
    }
    Q_FOREACH(Output* i, outputs_) {
        i->stop();
    }
    Q_FOREACH(Input* i, inputs_) {
        i->stop();
    }

    Q_FOREACH(Input* i, inputs_) {
        disconnectConnector(i);
    }
    Q_FOREACH(Output* i, outputs_) {
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
    QObject::connect(c, SIGNAL(connectionDone()), worker_, SLOT(checkIO()));
    QObject::connect(c, SIGNAL(connectionEnabled(bool)), worker_, SLOT(checkIO()));
    QObject::connect(c, SIGNAL(connectionRemoved()), worker_, SLOT(checkIO()));
}


void Node::disconnectConnector(Connectable */*c*/)
{
    //    QObject::disconnect(c, SIGNAL(connectionInProgress(Connectable*,Connectable*)), this, SIGNAL(connectionInProgress(Connectable*,Connectable*)));
    //    QObject::disconnect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    //    QObject::disconnect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
}
