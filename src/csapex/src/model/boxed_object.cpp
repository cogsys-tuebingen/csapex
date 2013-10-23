/// HEADER
#include <csapex/model/boxed_object.h>

/// COMPONENT
#include <csapex/model/box.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;

const BoxedObject::Ptr BoxedObject::NullPtr;

BoxedObject::BoxedObject()
{
}

//BoxedObject::BoxedObject(const std::string& type)
//    : type_(type), icon_(":/plugin.png"), enabled_(true)
//{
//    QMutexLocker lock(&active_mutex);

//    ++active_objects_;
//}

BoxedObject::~BoxedObject()
{
}


void BoxedObject::disable()
{
    Node::disable();
    setError(false);
}

void BoxedObject::errorEvent(bool error, ErrorLevel level)
{
    if(enabled_ && error && level == EL_ERROR) {
        box_->setIOError(true);
    } else {
        box_->setIOError(false);
    }
}
