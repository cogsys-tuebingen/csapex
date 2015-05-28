#ifndef NODE_CONSTRUCTOR_H
#define NODE_CONSTRUCTOR_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <utils_param/param_fwd.h>

/// SYSTEM
#include <typeinfo>
#include <functional>
#include <boost/signals2/signal.hpp>

namespace csapex
{

class NodeConstructor
{
    friend class command::AddNode;
    friend class NodeFactory;

public:
    struct NodeConstructionException : public std::runtime_error
    {
        NodeConstructionException(const std::string& what)
            : std::runtime_error(what)
        {
        }
    };

public:
    typedef std::shared_ptr<NodeConstructor> Ptr;

public:
    boost::signals2::signal<void()> unload_request;
    boost::signals2::signal<void()> reload_request;


public:
    NodeConstructor(const std::string& type, std::function<NodePtr()> c);

    virtual ~NodeConstructor();

    std::string getType() const;

    NodeConstructor& setTags(const std::vector<std::string> &tags);
    NodeConstructor& setTags(const std::vector<TagPtr> &tags);
    std::vector<TagPtr> getTags() const;

    NodeConstructor& setIcon(const std::string &icon);
    std::string getIcon() const;

    NodeConstructor& setDescription(const std::string &description);
    std::string getDescription() const;

    NodeWorkerPtr makePrototype() const;
    NodeWorkerPtr makeNodeWorker(const UUID& uuid) const;

    NodePtr makeNode() const;

protected:
    NodeConstructor(const std::string& type);

protected:
    std::string type_;
    std::string descr_;
    std::string icon_;
    std::vector<TagPtr> tags_;

    std::function<NodePtr()> c;
};

}
#endif // NODE_CONSTRUCTOR_H
