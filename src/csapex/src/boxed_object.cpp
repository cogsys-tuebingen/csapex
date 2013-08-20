/// HEADER
#include <csapex/boxed_object.h>

/// COMPONENT
#include <csapex/box.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;

BoxedObject::BoxedObject()
    : icon_(":/plugin.png"), enabled_(true)
{
}

BoxedObject::BoxedObject(const std::string& name)
    : name_(name), icon_(":/plugin.png"), enabled_(true)
{
}

BoxedObject::~BoxedObject()
{
}

bool BoxedObject::isEnabled()
{
    return enabled_;
}


void BoxedObject::setName(const std::string& name)
{
    name_ = name;
}

std::string BoxedObject::getName()
{
    return name_;
}


void BoxedObject::setTypeName(const std::string& type_name)
{
    type_name_ = type_name;
}

std::string BoxedObject::getTypeName()
{
    return type_name_;
}

void BoxedObject::setCategory(const std::string &category)
{
    if(!Tag::exists(category)) {
        Tag::create(category);
    }
    addTag(Tag::get(category));
}

void BoxedObject::addTag(const Tag &tag)
{
    tags_.push_back(tag);
}

std::vector<Tag> BoxedObject::getTags() const
{
    if(tags_.empty()) {
        tags_.push_back(Tag::get("General"));
    }
    return tags_;
}

void BoxedObject::setIcon(QIcon icon)
{
    icon_ = icon;
}

QIcon BoxedObject::getIcon()
{
    return icon_;
}

void BoxedObject::fill(QBoxLayout* layout)
{
    layout->addWidget(new QLabel(name_.c_str()));
}

void BoxedObject::updateDynamicGui(QBoxLayout *)
{
}

bool BoxedObject::canBeDisabled() const
{
    return true;
}

Memento::Ptr BoxedObject::getState() const
{
    return Memento::Ptr((Memento*) NULL);
}

void BoxedObject::setState(Memento::Ptr)
{

}

void BoxedObject::enable(bool e)
{
    enabled_ = e;
    if(e) {
        enable();
    } else {
        disable();
    }
}

void BoxedObject::enable()
{
    enabled_ = true;
    setError(false);
}

void BoxedObject::disable(bool d)
{
    enable(!d);
}


void BoxedObject::disable()
{
    enabled_ = false;
    setError(false);
}

void BoxedObject::connectorChanged()
{

}

void BoxedObject::tick()
{

}

void BoxedObject::errorEvent(bool error, ErrorLevel level)
{
    if(enabled_ && error && level == EL_ERROR) {
        box_->setIOError(true);
    } else {
        box_->setIOError(false);
    }
}
