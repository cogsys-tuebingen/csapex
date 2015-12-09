/// HEADER
#include <csapex/model/node_constructor.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/tag.h>
#include <csapex/utility/uuid.h>

using namespace csapex;

NodeConstructor::NodeConstructor(const std::string &type, std::function<NodePtr()> c)
    : type_(type), icon_(":/no_icon.png"), c(c)
{
}

NodeConstructor::NodeConstructor(const std::string &type)
    : type_(type), icon_(":/no_icon.png")
{
}

NodeConstructor::~NodeConstructor()
{
}


std::string NodeConstructor::getType() const
{
    return type_;
}

NodeConstructor& NodeConstructor::setTags(const std::vector<std::string> &strings)
{
    for(const std::string& name : strings) {
        tags_.push_back(Tag::get(name));
    }
    return *this;
}


NodeConstructor& NodeConstructor::setTags(const std::vector<TagPtr> &tags)
{
    tags_ = tags;
    return *this;
}

std::vector<TagPtr> NodeConstructor::getTags() const
{
    if(tags_.empty()) {
        return std::vector<TagPtr> { Tag::get("General") };
    }
    return tags_;
}



NodeConstructor& NodeConstructor::setIcon(const std::string& icon)
{
    icon_ = icon;
    return *this;
}

std::string NodeConstructor::getIcon() const
{
    return icon_.empty() ?  ":/plugin.png" : icon_;
}


NodeConstructor& NodeConstructor::setDescription(const std::string& description)
{
    descr_ = description;
    return *this;
}

std::string NodeConstructor::getDescription() const
{
    return descr_;
}

NodeWorker::Ptr NodeConstructor::makePrototype() const
{
    return makeNodeWorker(UUID::make("prototype"));
}

NodeWorker::Ptr NodeConstructor::makeNodeWorker(const UUID& uuid) const
{
    try {
        return std::make_shared<NodeWorker>(type_, uuid, makeNode());
    } catch(const std::exception& e) {
        std::cerr << "cannot construct node with UUID " << uuid.getFullName() << ": " << e.what() << std::endl;
        return nullptr;
    }
}

Node::Ptr NodeConstructor::makeNode() const
{
    return c();
}
