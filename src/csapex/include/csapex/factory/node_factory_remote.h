#ifndef NODE_FACTORY_REMOTE_H
#define NODE_FACTORY_REMOTE_H

/// PROJECT
#include <csapex/factory/node_factory.h>
#include <csapex/factory/node_factory_local.h>

namespace csapex
{

class CSAPEX_EXPORT NodeFactoryRemote: public NodeFactory
{
public:
    NodeFactoryRemote(NodeFactoryLocal& tmp_ref);


    virtual bool isValidType(const std::string& type) const override;

    virtual NodeConstructorPtr getConstructor(const std::string& type) override;
    virtual std::vector<NodeConstructorPtr> getConstructors() override;

    virtual std::map<std::string, std::vector<NodeConstructor::Ptr> > getTagMap() override;

private:
    NodeFactoryLocal& tmp_ref_;
};

}

#endif // NODE_FACTORY_REMOTE_H
