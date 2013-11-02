#ifndef TEMPLATE_CONSTRUCTOR_H
#define TEMPLATE_CONSTRUCTOR_H

/// COMPONENT
#include <csapex/model/node_constructor.h>

namespace csapex
{

class TemplateConstructor : public NodeConstructor
{
public:
    TemplateConstructor(bool temporary, const std::string& type, const std::string& description);

    virtual NodePtr makePrototypeContent() const;
    virtual NodePtr makeContent(const std::string& uuid) const;

protected:
    virtual void load() const;
};

}

#endif // TEMPLATE_CONSTRUCTOR_H
