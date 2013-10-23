/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/model/box.h>

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
