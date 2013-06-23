/// HEADER
#include "boxed_object.h"

/// COMPONENT
#include "box.h"

/// SYSTEM
#include <QLabel>
#include <QThread>

using namespace vision_evaluator;

BoxedObject::BoxedObject()
    : private_thread_(NULL), enabled_(true), error_(false)
{
}

BoxedObject::BoxedObject(const std::string& name)
    : name_(name), private_thread_(NULL), enabled_(true)
{
}

BoxedObject::~BoxedObject()
{
    if(private_thread_) {
        private_thread_->wait(1000);
        if(private_thread_->isRunning()) {
            std::cout << "terminate thread" << std::endl;
            private_thread_->terminate();
        }
    }
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
            if((i%line) == 0) {
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

void BoxedObject::stop()
{
    if(private_thread_) {
        private_thread_->quit();
        private_thread_->wait(1000);
        if(private_thread_->isRunning()) {
            std::cout << "terminate thread" << std::endl;
            private_thread_->terminate();
        }
    }
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

void BoxedObject::setBox(Box* box)
{
    box_ = box;
}

void BoxedObject::fill(QBoxLayout* layout)
{
    layout->addWidget(new QLabel(name_.c_str()));
}

void BoxedObject::makeThread()
{
    if(!private_thread_) {
        private_thread_ = new QThread;
    }
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
