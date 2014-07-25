/// HEADER
#include <csapex/model/node_constructor.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/core/settings.h>

/// SYSTEM
#include <boost/bind.hpp>

using namespace csapex;

NodePtr NodeConstructor::makeNull()
{
    return Node::Ptr (static_cast<Node*>(NULL));
}

NodeConstructor::NodeConstructor(Settings &settings,
                                 const std::string &type, const std::string &description,
                                 const std::string &icon,
                                 const std::vector<TagPtr>& tags,
                                 Make c)
    : settings_(settings), type_(type), descr_(description), icon_(icon), tags_(tags), c(c)
{
    apex_assert_hard(!c.empty());
}

NodeConstructor::NodeConstructor(Settings &settings,
                                 const std::string &type, const std::string &description,
                                 const std::string &icon,
                                 const std::vector<TagPtr>& tags)
    : settings_(settings), type_(type), descr_(description), icon_(icon), tags_(tags)
{
}

NodeConstructor::~NodeConstructor()
{
}


std::string NodeConstructor::getType() const
{
    return type_;
}

std::vector<Tag::Ptr> NodeConstructor::getTags() const
{
    return tags_;
}

QIcon NodeConstructor::getIcon() const
{
    return icon_.empty() ? QIcon(":/plugin.png") : QIcon(QString::fromStdString(icon_));
}

std::string NodeConstructor::getDescription() const
{
    return descr_;
}

Node::Ptr NodeConstructor::makePrototypeContent() const
{
    Node::Ptr res = c();
    res->setSettings(&settings_);
    res->setType(type_);
    res->setUUID(UUID::make("prototype"));

    res->doSetup();
    return res;
}

Node::Ptr NodeConstructor::makeContent(const UUID& uuid) const
{
    Node::Ptr res = c();
    res->setNodeWorker(new NodeWorker(res.get()));
    res->setSettings(&settings_);
    res->setType(type_);
    res->setUUID(uuid);
    res->getNodeState()->setLabel(uuid);

    res->doSetup();
    return res;
}
