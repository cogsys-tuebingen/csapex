#ifndef BOX_META_H
#define BOX_META_H

/// COMPONENT
#include <csapex/box.h>

namespace csapex
{

class BoxMeta : public Box
{
public:
    BoxMeta(BoxedObject* content, const std::string& uuid = "", QWidget* parent = 0);
};

}

#endif // BOX_META_H
