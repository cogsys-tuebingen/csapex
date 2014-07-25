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

NodeConstructor::NodeConstructor(Settings &settings, const std::string &type, const std::string &description, const std::string &icon, Make c)
    : settings_(settings), type_(type), descr_(description), icon_(icon), is_loaded(false), c(c)
{
    apex_assert_hard(!c.empty());
}

NodeConstructor::NodeConstructor(Settings &settings, const std::string &type, const std::string &description, const std::string &icon)
    : settings_(settings), type_(type), descr_(description), icon_(icon), is_loaded(false)
{
}

NodeConstructor::~NodeConstructor()
{
}


std::string NodeConstructor::getType() const
{
    return type_;
}

// todo: remove this, only read xml files!!!
void NodeConstructor::load() const
{
    try {
        Node::Ptr prototype = c();

        tags_ = prototype->getTags();
        prototype->setupParameters();

        is_loaded = true;

    } catch(const std::exception& e) {
        is_loaded = false;
        throw NodeConstructionException("cannot load object of type '" + type_ + "': " + e.what() );
    }
}

std::vector<Tag::Ptr> NodeConstructor::getTags() const
{
    if(!is_loaded) {
        load();
    }
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
