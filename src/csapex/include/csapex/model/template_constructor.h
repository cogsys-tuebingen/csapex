#ifndef TEMPLATE_CONSTRUCTOR_H
#define TEMPLATE_CONSTRUCTOR_H

/// COMPONENT
#include <csapex/model/boxed_object_constructor.h>

namespace csapex
{

class TemplateConstructor : public BoxedObjectConstructor
{
public:
    TemplateConstructor(const std::string& type, const std::string& description);

    virtual BoxedObjectPtr makePrototypeContent() const;
    virtual BoxedObjectPtr makeContent() const;

protected:
    virtual void load() const;
};

}

#endif // TEMPLATE_CONSTRUCTOR_H
