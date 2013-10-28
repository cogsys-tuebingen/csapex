/// HEADER
#include <csapex/model/boxed_object.h>

/// COMPONENT


/// SYSTEM
#include <QLabel>

using namespace csapex;

BoxedObject::BoxedObject(const std::string &uuid)
    : Node(uuid)
{
}

BoxedObject::~BoxedObject()
{
}
