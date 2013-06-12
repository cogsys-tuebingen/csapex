/// HEADER
#include "boxed_object.h"

/// COMPONENT
#include "box.h"

/// SYSTEM
#include <QLabel>

using namespace vision_evaluator;

BoxedObject::BoxedObject()
{
}

BoxedObject::BoxedObject(const std::string& name)
    : name_(name)
{
}

BoxedObject::~BoxedObject()
{
}

void BoxedObject::setName(const std::string &name)
{
    name_ = name;
}

std::string BoxedObject::getName()
{
    return name_;
}

void BoxedObject::setBox(Box *box)
{
    box_ = box;
}

void BoxedObject::fill(QBoxLayout *layout)
{
    layout->addWidget(new QLabel(name_.c_str()));
}
