/// HEADER
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;

BoxedObject::BoxedObject(const UUID &uuid)
    : Node(uuid)
{
}

BoxedObject::~BoxedObject()
{
}

void BoxedObject::setupUi(QBoxLayout *layout)
{
    fill(layout);
}

void BoxedObject::setup()
{
}

void BoxedObject::fill(QBoxLayout *)
{
}
