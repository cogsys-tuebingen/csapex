/// HEADER
#include <csapex/model/node_constructor.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/model/tag.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/utility/delegate_bind.h>

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

NodeHandlePtr NodeConstructor::makePrototype() const
{
    return makeNodeHandle(UUIDProvider::makeUUID_forced("prototype"), nullptr);
}

NodeHandlePtr NodeConstructor::makeNodeHandle(const UUID& uuid, csapex::UUIDProvider *uuid_provider) const
{
    try {
        OutputTransitionPtr ot = std::make_shared<OutputTransition>();
        InputTransitionPtr it = std::make_shared<InputTransition>();
        NodeHandlePtr node_handle = std::make_shared<NodeHandle>(type_, uuid, makeNode(), uuid_provider, it, ot);
        if(uuid_provider) {
            uuid_provider->registerUUID(uuid);
        }
        return node_handle;

    } catch(const std::exception& e) {
        std::cerr << "cannot construct node with UUID " << uuid.getFullName() << ": " << e.what() << std::endl;
        return nullptr;
    }
}

Node::Ptr NodeConstructor::makeNode() const
{
    return c();
}
