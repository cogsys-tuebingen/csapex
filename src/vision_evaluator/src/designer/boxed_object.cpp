/// HEADER
#include "boxed_object.h"

/// COMPONENT
#include "box.h"

/// SYSTEM
#include <QLabel>
#include <QThread>

using namespace vision_evaluator;

BoxedObject::BoxedObject()
    : private_thread_(NULL)
{
}

BoxedObject::BoxedObject(const std::string& name)
    : name_(name), private_thread_(NULL)
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

Memento::Ptr BoxedObject::saveState()
{
    return Memento::Ptr((Memento*) NULL);
}

void BoxedObject::loadState(Memento::Ptr memento)
{

}
