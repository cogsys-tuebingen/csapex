/// HEADER
#include "boxed_object.h"

/// COMPONENT
#include "box.h"

/// SYSTEM
#include <QLabel>

using namespace vision_evaluator;

BoxedObject::BoxedObject()
    : enabled_(true), error_(false)
{
}

BoxedObject::BoxedObject(const std::string& name)
    : name_(name), enabled_(true)
{
}

BoxedObject::~BoxedObject()
{
}

bool BoxedObject::isEnabled()
{
    return enabled_;
}

void BoxedObject::setError(bool e, const std::string& msg)
{
    QString err;
    if(e) {
        unsigned line = 60;
        for(unsigned i = 0; i < msg.size(); ++i) {
            err += msg[i];
            if((i%line) == 0 && i != 0) {
                err += '\n';
            }
        }
    }
    box_->setToolTip(err);
    error_ = e;
}
bool BoxedObject::isError()
{
    return error_;
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
    category_ = category;
}

std::string BoxedObject::getCategory()
{
    return category_;
}

void BoxedObject::setBox(Box* box)
{
    box_ = box;
}

void BoxedObject::fill(QBoxLayout* layout)
{
    layout->addWidget(new QLabel(name_.c_str()));
}

void BoxedObject::updateDynamicGui(QBoxLayout *layout)
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

void BoxedObject::setState(Memento::Ptr memento)
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
