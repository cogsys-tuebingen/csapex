/// HEADER
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;

BoxedObject::BoxedObject(const UUID &uuid)
    : Node(uuid), NodeAdapter(this, NULL)
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
