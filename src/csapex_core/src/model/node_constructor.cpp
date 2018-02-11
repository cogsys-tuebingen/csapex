/// HEADER
#include <csapex/model/node_constructor.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/tag.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/delegate_bind.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace csapex;

NodeConstructor::NodeConstructionException::NodeConstructionException(const std::string& what)
    : std::runtime_error(what)
{
}

NodeConstructor::NodeConstructor(const std::string &type, std::function<NodePtr()> c)
    : type_(type), icon_(":/no_icon.png"), properties_loaded_(false), c(c)
{
}

NodeConstructor::NodeConstructor(const std::string &type)
    : type_(type), icon_(":/no_icon.png"), properties_loaded_(false)
{
}

NodeConstructor::NodeConstructor()
    : type_("unnamed"), icon_(":/no_icon.png"), properties_loaded_(false)
{
}


NodeConstructor::~NodeConstructor()
{
}


std::string NodeConstructor::getType() const
{
    return type_;
}

NodeConstructor& NodeConstructor::setTags(const std::string &taglist)
{
    // convert tag list into vector
    std::vector<std::string> tokens;

    boost::algorithm::split(tokens, taglist, boost::is_any_of(",;"));

    for(std::vector<std::string>::const_iterator it = tokens.begin(); it != tokens.end(); ++it) {
        std::string str = boost::algorithm::trim_copy(*it);
        if(!str.empty()) {
            tags_.push_back(Tag::get(str));
        }
    }

    return *this;
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

NodeConstructor& NodeConstructor::setProperties(const std::vector<std::string> &properties)
{
    properties_ = properties;
    properties_loaded_ = true;

    return *this;
}

std::vector<std::string> NodeConstructor::getProperties() const
{
    if(!properties_loaded_) {
        try {
            NodePtr node = makeNode();
            node->getProperties(properties_);

        } catch(...) {
            // ignore error and mark the node as invalid
            properties_.push_back("invalid");
        }

        properties_loaded_ = true;
    }
    return properties_;
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
    return makeNodeHandle(UUIDProvider::makeUUID_without_parent("prototype"), nullptr);
}

NodeHandlePtr NodeConstructor::makeNodeHandle(const UUID& uuid, const UUIDProviderPtr& uuid_provider) const
{
    try {
        NodePtr node = makeNode();
        OutputTransitionPtr ot = std::make_shared<OutputTransition>();
        InputTransitionPtr it = std::make_shared<InputTransition>();
        NodeHandlePtr node_handle = std::make_shared<NodeHandle>(type_, uuid, node, uuid_provider, it, ot);
        node->initialize(node_handle);

        if(!uuid.empty() && uuid_provider) {
            uuid_provider->registerUUID(uuid);
        }
        return node_handle;

    } catch(const std::exception& e) {
        std::cerr << "cannot construct node with UUID " << uuid.getFullName() << ": " << e.what() << std::endl;
        return nullptr;
    }
}

std::vector<csapex::param::ParameterPtr> NodeConstructor::getParameters() const
{
    if(auto node = makePrototype()->getNode().lock()) {
        node->setupParameters(*node);
        return node->getParameters();
    }

    return {};
}

Node::Ptr NodeConstructor::makeNode() const
{
    return c();
}


void NodeConstructor::serialize(SerializationBuffer &data) const
{
    data << type_;
    data << descr_;
    data << icon_;
    data << tags_;
    data << properties_;
    data << true;
}
void NodeConstructor::deserialize(const SerializationBuffer& data)
{
    data >> type_;
    data >> descr_;
    data >> icon_;
    data >> tags_;
    data >> properties_;
    data >> properties_loaded_;
}

std::shared_ptr<Clonable> NodeConstructor::makeEmptyClone() const
{
    return std::shared_ptr<Clonable>(new NodeConstructor(""));
}
