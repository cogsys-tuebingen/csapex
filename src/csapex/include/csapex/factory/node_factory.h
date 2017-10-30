#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

/// PROJECT
#include <csapex/utility/notifier.h>
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <vector>
#include <string>
#include <map>

class TiXmlElement;

namespace csapex
{

class CSAPEX_EXPORT NodeFactory : public Notifier
{
public:
    typedef std::shared_ptr<NodeFactory> Ptr;

public:
    virtual bool isValidType(const std::string& type) const = 0;

    virtual NodeConstructorPtr getConstructor(const std::string& type) = 0;
    virtual std::vector<NodeConstructorPtr> getConstructors() = 0;

    virtual std::map<std::string, std::vector<NodeConstructorPtr> > getTagMap() = 0;
};

}

#endif // NODE_FACTORY_H
