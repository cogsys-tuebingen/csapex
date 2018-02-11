#ifndef NODE_FACTORY_H
#define NODE_FACTORY_H

/// PROJECT
#include <csapex/model/notifier.h>
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <vector>
#include <string>
#include <map>

namespace csapex
{

class CSAPEX_CORE_EXPORT NodeFactory : public Notifier
{
public:
    typedef std::shared_ptr<NodeFactory> Ptr;

public:
    bool isValidType(const std::string& type);

    NodeConstructorPtr getConstructor(const std::string& type);

    std::vector<NodeConstructorPtr> getConstructors();

    std::map<std::string, std::vector<NodeConstructorPtr> > getTagMap();

protected:
    virtual void ensureLoaded() = 0;

protected:
    std::map<std::string, std::vector<NodeConstructorPtr> > tag_map_;
    std::vector<NodeConstructorPtr> constructors_;
};

}

#endif // NODE_FACTORY_H
