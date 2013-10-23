/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;

Node::Node()
    : icon_(":/plugin.png"), enabled_(true)
{

}

Node::~Node()
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
    return enabled_;
}
void Node::messageArrived(ConnectorIn *)
{

}
void Node::allConnectorsArrived()
{

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
    enabled_ = e;
    if(e) {
        enable();
    } else {
        disable();
    }
}

void Node::enable()
{
    enabled_ = true;
}

void Node::disable(bool d)
{
    enable(!d);
}


void Node::disable()
{
    enabled_ = false;
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


void Node::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    box_->setError(error, msg, level);
    if(enabled_ && error && level == EL_ERROR) {
        box_->setIOError(true);
    } else {
        box_->setIOError(false);
    }
}



ConnectorIn* Node::addInput(ConnectionTypePtr type, const std::string& label, bool optional)
{
    int id = input.size();
    ConnectorIn* c = new ConnectorIn(box_, id);
    c->setLabel(label);
    c->setOptional(optional);
    c->setType(type);

    registerInput(c);

    return c;
}

ConnectorOut* Node::addOutput(ConnectionTypePtr type, const std::string& label)
{
    int id = output.size();
    ConnectorOut* c = new ConnectorOut(box_, id);
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
    box_->setSynchronizedInputs(sync);
}

int Node::countInputs()
{
    return box_->countInputs();
}

int Node::countOutputs()
{
    return box_->countOutputs();
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

    Q_EMIT connectorRemoved(in);
}

void Node::removeOutput(ConnectorOut *out)
{
    std::vector<ConnectorOut*>::iterator it;
    it = std::find(output.begin(), output.end(), out);

    assert(*it == out);

    out->deleteLater();
    output.erase(it);

    Q_EMIT connectorRemoved(out);
}


void Node::registerInput(ConnectorIn* in)
{
    input.push_back(in);

    Q_EMIT connectorCreated(in);
}

void Node::registerOutput(ConnectorOut* out)
{
    output.push_back(out);

    Q_EMIT connectorCreated(out);
}
