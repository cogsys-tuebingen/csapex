/// HEADER
#include <csapex/model/boxed_object.h>

/// COMPONENT
#include <csapex/model/box.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;

int BoxedObject::active_objects_ = 0;
QMutex BoxedObject::active_mutex;

const BoxedObject::Ptr BoxedObject::NullPtr;

BoxedObject::BoxedObject()
    : enabled_(true)
{
    QMutexLocker lock(&active_mutex);

    ++active_objects_;
}

//BoxedObject::BoxedObject(const std::string& type)
//    : type_(type), icon_(":/plugin.png"), enabled_(true)
//{
//    QMutexLocker lock(&active_mutex);

//    ++active_objects_;
//}

BoxedObject::~BoxedObject()
{
    QMutexLocker lock(&active_mutex);

    --active_objects_;
    if(active_objects_ == 0) {
//        std::cout << "destroyed BoxedObject, active objects left: " << active_objects_ << std::endl;
    }
}

bool BoxedObject::isEnabled()
{
    return enabled_;
}


void BoxedObject::updateModel()
{
}

bool BoxedObject::canBeDisabled() const
{
    return true;
}

void BoxedObject::messageArrived(ConnectorIn *)
{

}
void BoxedObject::allConnectorsArrived()
{

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
