/// HEADER
#include <csapex/model/node_constructor.h>

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <boost/bind.hpp>

using namespace csapex;

NodePtr NodeConstructor::makeNull()
{
    return Node::Ptr (new Node);
}

NodeConstructor::NodeConstructor(const std::string &type, const std::string &description, Make c)
    : type_(type), descr_(description), is_loaded(false), c(c)
{
    assert(!c.empty());
}

NodeConstructor::NodeConstructor(const std::string &type, const std::string &description)
    : type_(type), descr_(description), is_loaded(false)
{
}

NodeConstructor::~NodeConstructor()
{
}


std::string NodeConstructor::getType() const
{
    return type_;
}

void NodeConstructor::load() const
{
    try {
        Node::Ptr prototype = c();

        icon = prototype->getIcon();
        cat = prototype->getTags();

        is_loaded = true;

    } catch(const std::exception& e) {
        is_loaded = false;
        throw NodeConstructionException("cannot load object of type '" + type_ + "': " + e.what() );
    }
}

std::vector<Tag> NodeConstructor::getTags() const
{
    if(!is_loaded) {
        load();
    }
    return cat;
}

QIcon NodeConstructor::getIcon() const
{
    return icon;
}

std::string NodeConstructor::getDescription() const
{
    return descr_;
}

Node::Ptr NodeConstructor::makePrototypeContent() const
{
    Node::Ptr res = c();
    res->setType(type_);
    return res;
}

Node::Ptr NodeConstructor::makeContent(const UUID& uuid) const
{
    Node::Ptr res = c();
    res->setType(type_);
    res->setUUID(uuid);
    res->setLabel(uuid);
    return res;
}
