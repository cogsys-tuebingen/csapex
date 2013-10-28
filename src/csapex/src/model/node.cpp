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
      uuid_(uuid), state(new NodeState(this)), dispatcher_(NULL), loaded_state_available_(false)
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

std::string Node::getType()
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

void Node::setIcon(QIcon icon)
{
    icon_ = icon;
}

QIcon Node::getIcon()
{
    return icon_;
}

bool Node::canBeDisabled() const
{
    return true;
}

bool Node::isEnabled()
{
    return state->enabled;
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
    assert(state);

    NodeState::Ptr memento(new NodeState(this));
    *memento = *state;

    memento->boxed_state = getState();

    return memento;
}

void Node::setNodeState(NodeState::Ptr memento)
{
    boost::shared_ptr<NodeState> m = boost::dynamic_pointer_cast<NodeState> (memento);
    assert(m.get());

    std::string old_uuid = UUID();
    std::string old_label = state->label_;

    *state = *m;

    if(state->label_.empty()) {
        state->label_ = old_label;
    }
    if(uuid_.empty()) {
        uuid_ = old_uuid;
    }

    state->parent = this;
    if(m->boxed_state != NULL) {
        setState(m->boxed_state);
    }

    Q_EMIT stateChanged();
}

void Node::setNodeStateLater(NodeStatePtr s)
{
    *state = *s;
    //    setNodeState(s);
    loaded_state_available_ = true;
}

Memento::Ptr Node::getState() const
{
    return Memento::Ptr((Memento*) NULL);
}

void Node::setState(Memento::Ptr)
{

}

void Node::enable(bool e)
{
    state->enabled = e;
    if(e) {
        enable();
    } else {
        disable();
    }
}

void Node::enable()
{
    state->enabled = true;
}

void Node::disable(bool d)
{
    enable(!d);
}


void Node::disable()
{
    state->enabled = false;
    setError(false);
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
        setNodeState(state);
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
    if(state->enabled && error && level == EL_ERROR) {
        box_->setIOError(true);
    } else {
        box_->setIOError(false);
    }
}



ConnectorIn* Node::addInput(ConnectionTypePtr type, const std::string& label, bool optional)
{
    int id = input.size();
    ConnectorIn* c = new ConnectorIn(this, id);
    c->setLabel(label);
    c->setOptional(optional);
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

int Node::countInputs()
{
    return input.size();
}

int Node::countOutputs()
{
    return output.size();
}

ConnectorIn* Node::getInput(const unsigned int index)
{
    assert(index < input.size());
    return input[index];
}

ConnectorOut* Node::getOutput(const unsigned int index)
{
    assert(index < output.size());
    return output[index];
}

ConnectorIn* Node::getInput(const std::string& uuid)
{
    BOOST_FOREACH(ConnectorIn* in, input) {
        if(in->UUID() == uuid) {
            return in;
        }
    }

    return NULL;
}

ConnectorOut* Node::getOutput(const std::string& uuid)
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
    command::Meta::Ptr cmd(new command::Meta);

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
    state->pos = pos;
}

QPoint Node::getPosition() const
{
    return state->pos;
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
    state->writeYaml(out);

    return out;
}

void Node::read(const YAML::Node &doc)
{
    NodeState::Ptr s = getNodeState();
    s->readYaml(doc);

    setNodeStateLater(s);
}
