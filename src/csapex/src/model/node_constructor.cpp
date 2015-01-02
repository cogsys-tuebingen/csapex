/// HEADER
#include <csapex/model/node_constructor.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/core/settings.h>
#include <csapex/model/tag.h>

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
    : unload_request(new boost::signals2::signal<void()>),
      reload_request(new boost::signals2::signal<void()>),
      settings_(settings), type_(type), descr_(description), icon_(icon), tags_(tags), c(c)
{
    apex_assert_hard(!c.empty());

    if(tags_.empty()) {
        tags_.push_back(Tag::get("General"));
    }
}

NodeConstructor::NodeConstructor(Settings &settings,
                                 const std::string &type, const std::string &description,
                                 const std::string &icon,
                                 const std::vector<TagPtr>& tags)
    : unload_request(new boost::signals2::signal<void()>),
      reload_request(new boost::signals2::signal<void()>),
      settings_(settings), type_(type), descr_(description), icon_(icon), tags_(tags)
{
    if(tags_.empty()) {
        tags_.push_back(Tag::get("General"));
    }
}

NodeConstructor::~NodeConstructor()
{
}


std::string NodeConstructor::getType() const
{
    return type_;
}

std::vector<TagPtr> NodeConstructor::getTags() const
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

NodeWorker::Ptr NodeConstructor::makePrototype() const
{
    return makeNodeWorker(UUID::make("prototype"));
}

NodeWorker::Ptr NodeConstructor::makeNodeWorker(const UUID& uuid) const
{
    try {
        NodeWorker::Ptr result(new NodeWorker(type_, uuid, settings_, makeNode()));
        return result;
    } catch(const std::exception& e) {
        std::cerr << "cannot construct node with UUID " << uuid.getFullName() << ": " << e.what() << std::endl;
        return NodeWorkerNullPtr;
    }
}

Node::Ptr NodeConstructor::makeNode() const
{
    return c();
}
