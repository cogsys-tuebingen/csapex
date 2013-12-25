/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QThread>

using namespace csapex;

Node::Node(const std::string &uuid)
    : icon_(":/plugin.png"), box_(NULL), private_thread_(NULL), worker_(new NodeWorker(this)),
      uuid_(uuid), node_state_(new NodeState(this)), dispatcher_(NULL), loaded_state_available_(false)
{
    QObject::connect(worker_, SIGNAL(messageProcessed()), this, SLOT(messageProcessed()));
}

Node::~Node()
{
    delete worker_;
}

void Node::makeThread()
{
    if(!private_thread_) {
        private_thread_ = new QThread;
        connect(private_thread_, SIGNAL(finished()), private_thread_, SLOT(deleteLater()));

        assert(worker_);
        worker_->moveToThread(private_thread_);

        private_thread_->start();
    }
}

void Node::setUUID(const std::string &uuid)
{
    uuid_ = uuid;
}

std::string Node::UUID() const
{
    return uuid_;
}

void Node::setup()
{

}

void Node::setType(const std::string &type)
{
    type_ = type;
}

std::string Node::getType() const
{
    return type_;
}

void Node::setCategory(const std::string &category)
{
    if(!Tag::exists(category)) {
        Tag::create(category);
    }
    addTag(Tag::get(category));
}

void Node::addTag(const Tag &tag)
{
    tags_.push_back(tag);
}

std::vector<Tag> Node::getTags() const
{
    if(tags_.empty()) {
        tags_.push_back(Tag::get("General"));
    }
    return tags_;
}

void Node::addParameter(const param::Parameter::Ptr &param)
{
    state.params[param->name()] = param;
}

void Node::addParameter(const param::Parameter::Ptr &param, boost::function<void (param::Parameter *)> cb)
{
    addParameter(param);
    worker_->addParameterCallback(param, cb);
}

std::vector<param::Parameter::Ptr> Node::getParameters() const
{
    std::vector<param::Parameter::Ptr> r;
    for( std::map<std::string, param::Parameter::Ptr>::const_iterator it = state.params.begin(); it != state.params.end(); ++it ) {
        r.push_back( it->second );
    }
    return r;
}

param::Parameter::Ptr Node::getParameter(const std::string &name) const
{
    return state.params.at(name);
}

void Node::setIcon(QIcon icon)
{
    icon_ = icon;
}

QIcon Node::getIcon() const
{
    return icon_;
}

bool Node::canBeDisabled() const
{
    return true;
}

bool Node::isEnabled()
{
    return node_state_->enabled;
}
void Node::messageArrived(ConnectorIn *)
{

}
void Node::allConnectorsArrived()
{

}

void Node::messageProcessed()
{
    foreach(ConnectorIn* i, input) {
        i->notify();
    }
}

void Node::killContent()
{
    if(private_thread_ && private_thread_->isRunning()) {

        QMutexLocker lock(&worker_mutex_);

        QObject::disconnect(private_thread_);
        QObject::disconnect(worker_);

        QObject::connect(private_thread_, SIGNAL(finished()), private_thread_, SLOT(deleteLater()));
        QObject::connect(private_thread_, SIGNAL(terminated()), private_thread_, SLOT(deleteLater()));

        QObject::connect(private_thread_, SIGNAL(finished()), worker_, SLOT(deleteLater()));
        QObject::connect(private_thread_, SIGNAL(terminated()), worker_, SLOT(deleteLater()));

        private_thread_->quit();
        if(!private_thread_->wait(100)) {
            private_thread_->terminate();
        }

        private_thread_ = NULL;
        worker_ = new NodeWorker(this);

        BOOST_FOREACH(ConnectorIn* in, input) {
            QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), worker_, SLOT(forwardMessage(ConnectorIn*)));
        }

        makeThread();
    }
}

NodeState::Ptr Node::getNodeState()
{
    assert(node_state_);

    NodeState::Ptr memento(new NodeState(this));
    *memento = *node_state_;

    memento->boxed_state = getState();

    return memento;
}

void Node::setNodeState(NodeState::Ptr memento)
{
    boost::shared_ptr<NodeState> m = boost::dynamic_pointer_cast<NodeState> (memento);
    assert(m.get());

    std::string old_uuid = UUID();
    std::string old_label = node_state_->label_;

    *node_state_ = *m;

    if(node_state_->label_.empty()) {
        node_state_->label_ = old_label;
    }
    if(uuid_.empty()) {
        uuid_ = old_uuid;
    }

    node_state_->parent = this;
    if(m->boxed_state != NULL) {
        setState(m->boxed_state);
    }

    Q_EMIT stateChanged();
}

void Node::setNodeStateLater(NodeStatePtr s)
{
    *node_state_ = *s;
    boost::shared_ptr<GenericState> m = boost::dynamic_pointer_cast<GenericState> (s->boxed_state);
    if(m) {
        for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = m->params.begin(); it != m->params.end(); ++it ) {
            param::Parameter* param = it->second.get();
            if(state.params.find(param->name()) != state.params.end()) {
                state.params[param->name()]->setFrom(*param);
            } else {
                std::cout << "warning: parameter " << param->name() << " is ignored!" << std::endl;
            }
        }
    }
    loaded_state_available_ = true;
}

Memento::Ptr Node::getState() const
{
    return GenericState::Ptr(new GenericState(state));
}

void Node::setState(Memento::Ptr memento)
{
    boost::shared_ptr<GenericState> m = boost::dynamic_pointer_cast<GenericState> (memento);
    assert(m.get());

    std::map<std::string, param::Parameter::Ptr> old_params = state.params;
    state = *m;
    state.params = old_params;
    for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = m->params.begin(); it != m->params.end(); ++it) {
        param::Parameter::Ptr p = it->second;
        if(state.params.find(p->name()) != state.params.end()) {
            state.params[p->name()]->setFrom(*p);
        }
    }

    Q_EMIT modelChanged();
}

void Node::enable(bool e)
{
    node_state_->enabled = e;
    enableIO(e);
    if(e) {
        enable();
    } else {
        disable();
    }
}

void Node::enable()
{
    node_state_->enabled = true;

    if(box_) {
        box_->enabledChange(node_state_->enabled);
    }
}

void Node::disable(bool d)
{
    enable(!d);
}


void Node::disable()
{
    node_state_->enabled = false;
    setError(false);

    if(box_) {
        box_->enabledChange(node_state_->enabled);
    }
}

void Node::enableIO(bool enable)
{
    foreach(ConnectorIn* i, input) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
    foreach(ConnectorOut* i, output) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}

void Node::setIOError(bool error)
{
    foreach(ConnectorIn* i, input) {
        i->setErrorSilent(error);
    }
    foreach(ConnectorOut* i, output) {
        i->setErrorSilent(error);
    }
    enableIO(!error);
}

void Node::setLabel(const std::string &label)
{
    node_state_->label_ = label;
}

void Node::setMinimized(bool min)
{
    node_state_->minimized = min;
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

void Node::eventGuiChanged()
{
    worker_->eventGuiChanged();

    if(loaded_state_available_) {
        loaded_state_available_ = false;
        setNodeState(node_state_);
    }
}


void Node::setBox(Box* box)
{
    QMutexLocker lock(&mutex);
    box_ = box;
}

Box* Node::getBox() const
{
    QMutexLocker lock(&mutex);
    return box_;
}

NodeWorker* Node::getNodeWorker() const
{
    return worker_;
}


void Node::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    box_->setError(error, msg, level);
    if(node_state_->enabled && error && level == EL_ERROR) {
        setIOError(true);
    } else {
        setIOError(false);
    }
}



ConnectorIn* Node::addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async)
{
    int id = input.size();
    ConnectorIn* c = new ConnectorIn(this, id);
    c->setLabel(label);
    c->setOptional(optional);
    c->setAsync(async);
    c->setType(type);

    registerInput(c);

    return c;
}

ConnectorOut* Node::addOutput(ConnectionTypePtr type, const std::string& label)
{
    int id = output.size();
    ConnectorOut* c = new ConnectorOut(this, id);
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

void Node::setSynchronizedInputs(bool sync)
{
    worker_->setSynchronizedInputs(sync);
}

int Node::countInputs() const
{
    return input.size();
}

int Node::countOutputs() const
{
    return output.size();
}

ConnectorIn* Node::getInput(const unsigned int index) const
{
    assert(index < input.size());
    return input[index];
}

ConnectorOut* Node::getOutput(const unsigned int index) const
{
    assert(index < output.size());
    return output[index];
}

ConnectorIn* Node::getInput(const std::string& uuid) const
{
    BOOST_FOREACH(ConnectorIn* in, input) {
        if(in->UUID() == uuid) {
            return in;
        }
    }

    return NULL;
}

ConnectorOut* Node::getOutput(const std::string& uuid) const
{
    BOOST_FOREACH(ConnectorOut* out, output) {
        if(out->UUID() == uuid) {
            return out;
        }
    }

    return NULL;
}
void Node::removeInput(ConnectorIn *in)
{
    std::vector<ConnectorIn*>::iterator it;
    it = std::find(input.begin(), input.end(), in);

    assert(*it == in);

    in->deleteLater();
    input.erase(it);

    disconnectConnector(in);
    Q_EMIT connectorRemoved(in);
}

void Node::removeOutput(ConnectorOut *out)
{
    std::vector<ConnectorOut*>::iterator it;
    it = std::find(output.begin(), output.end(), out);

    assert(*it == out);

    out->deleteLater();
    output.erase(it);

    disconnectConnector(out);
    Q_EMIT connectorRemoved(out);
}


Command::Ptr Node::removeAllConnectionsCmd()
{
    command::Meta::Ptr cmd(new command::Meta("Remove All Connectors"));

    BOOST_FOREACH(ConnectorIn* i, input) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }
    BOOST_FOREACH(ConnectorOut* i, output) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }

    return cmd;
}

QTreeWidgetItem * Node::createDebugInformationConnector(Connector* connector) const
{
    QTreeWidgetItem* connector_widget = new QTreeWidgetItem;
    connector_widget->setText(0, "Connector");
    connector_widget->setIcon(0, QIcon(":/connector.png"));

    QTreeWidgetItem* uuid = new QTreeWidgetItem;
    uuid->setText(0, "UUID");
    uuid->setText(1, connector->UUID().c_str());
    connector_widget->addChild(uuid);

    QTreeWidgetItem* label = new QTreeWidgetItem;
    label->setText(0, "Label");
    label->setText(1, connector->getLabel().c_str());
    connector_widget->addChild(label);


    QTreeWidgetItem* type = new QTreeWidgetItem;
    type->setText(0, "Type");
    type->setText(1, connector->getType()->name().c_str());
    connector_widget->addChild(type);

    return connector_widget;
}

QTreeWidgetItem* Node::createDebugInformation() const
{
    QTreeWidgetItem* tl = new QTreeWidgetItem;
    tl->setText(0, UUID().c_str());
    tl->setIcon(0, getIcon());

    {
        QTreeWidgetItem* connectors = new QTreeWidgetItem;
        connectors->setText(0, "Inputs");
        for(int i = 0, n = countInputs(); i < n; ++i) {
            ConnectorIn* connector = getInput(i);

            QTreeWidgetItem* connector_widget = createDebugInformationConnector(connector);

            QTreeWidgetItem* input = new QTreeWidgetItem;
            input->setText(0, "Input");

            QTreeWidgetItem* target_widget = new QTreeWidgetItem;
            if(connector->isConnected()) {
                Connector* target = connector->getSource();
                target_widget->setText(0, target->UUID().c_str());
                target_widget->setIcon(1, target->getNode()->getIcon());
                target_widget->setText(1, target->getNode()->getType().c_str());
            } else {
                target_widget->setText(0, "not connected");
                target_widget->setIcon(1, QIcon(":/disconnected.png"));
            }
            input->addChild(target_widget);

            connector_widget->addChild(input);

            connectors->addChild(connector_widget);
        }
        tl->addChild(connectors);
    }
    {
        QTreeWidgetItem* connectors = new QTreeWidgetItem;
        connectors->setText(0, "Outputs");
        for(int i = 0, n = countOutputs(); i < n; ++i) {
            ConnectorOut* connector = getOutput(i);

            QTreeWidgetItem* connector_widget = createDebugInformationConnector(connector);

            QTreeWidgetItem* targets = new QTreeWidgetItem;
            targets->setText(0, "Target");
            for(ConnectorOut::TargetIterator it = connector->beginTargets(); it != connector->endTargets(); ++it) {
                ConnectorIn* target = *it;
                QTreeWidgetItem* target_widget = new QTreeWidgetItem;
                target_widget->setText(0, target->UUID().c_str());
                target_widget->setIcon(1, target->getNode()->getIcon());
                target_widget->setText(1, target->getNode()->getType().c_str());
                targets->addChild(target_widget);
            }
            connector_widget->addChild(targets);

            connectors->addChild(connector_widget);
        }
        tl->addChild(connectors);
    }

    return tl;
}

void Node::registerInput(ConnectorIn* in)
{
    input.push_back(in);

    worker_->addInput(in);
    connectConnector(in);
    QObject::connect(in, SIGNAL(messageArrived(Connector*)), worker_, SLOT(forwardMessage(Connector*)));

    Q_EMIT connectorCreated(in);
}

void Node::registerOutput(ConnectorOut* out)
{
    output.push_back(out);

    connectConnector(out);

    Q_EMIT connectorCreated(out);
}

int Node::nextInputId()
{
    return input.size();
}

int Node::nextOutputId()
{
    return output.size();
}

void Node::setPosition(const QPoint &pos)
{
    node_state_->pos = pos;
}

QPoint Node::getPosition() const
{
    return node_state_->pos;
}

CommandDispatcher* Node::getCommandDispatcher() const
{
    return dispatcher_;
}

void Node::setCommandDispatcher(CommandDispatcher *d)
{
    dispatcher_ = d;
}

void Node::stop()
{
    foreach(ConnectorIn* i, input) {
        disconnectConnector(i);
    }
    foreach(ConnectorOut* i, output) {
        disconnectConnector(i);
    }

    QObject::disconnect(private_thread_);
    QObject::disconnect(worker_);
    QObject::disconnect(this);

    if(private_thread_) {
        private_thread_->quit();
        private_thread_->wait(1000);
        if(private_thread_->isRunning()) {
            std::cout << "terminate thread" << std::endl;
            private_thread_->terminate();
        }
    }
}

void Node::connectConnector(Connector *c)
{
    QObject::connect(c, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::connect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    QObject::connect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
}


void Node::disconnectConnector(Connector *c)
{
    QObject::disconnect(c, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::disconnect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    QObject::disconnect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
}


YAML::Emitter& Node::save(YAML::Emitter& out) const
{
    node_state_->writeYaml(out);

    return out;
}

void Node::read(const YAML::Node &doc)
{
    NodeState::Ptr s = getNodeState();
    s->readYaml(doc);

    setNodeStateLater(s);
}
