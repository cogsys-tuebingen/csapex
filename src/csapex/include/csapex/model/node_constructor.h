#ifndef NODE_CONSTRUCTOR_H
#define NODE_CONSTRUCTOR_H

/// COMPONENT
#include <csapex/utility/constructor.hpp>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>
#include <utils_param/param_fwd.h>
#include <csapex/plugin/plugin_constructor.hpp>

/// SYSTEM
#include <typeinfo>
#include <functional>

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
    typedef std::function<NodePtr()> Make;

    typedef std::shared_ptr<NodeConstructor> Ptr;

public:
    std::shared_ptr< boost::signals2::signal<void()> > unload_request;
    std::shared_ptr< boost::signals2::signal<void()> > reload_request;


public:
    NodeConstructor(Settings& settings, const std::string& type,
                    const std::string& description, const std::string& icon, const std::vector<TagPtr> &tags,
                    Make c);

    virtual ~NodeConstructor();

    std::string getType() const;
    std::vector<TagPtr> getTags() const;
    std::string getIcon() const;
    std::string getDescription() const;

    NodeWorkerPtr makePrototype() const;
    NodeWorkerPtr makeNodeWorker(const UUID& uuid) const;

    NodePtr makeNode() const;

protected:
    NodeConstructor(Settings &settings, const std::string& type, const std::string& description, const std::string &icon, const std::vector<TagPtr> &tags);

protected:
    Settings& settings_;
    std::string type_;
    std::string descr_;
    std::string icon_;
    std::vector<TagPtr> tags_;

    Make c;
};

}
#endif // NODE_CONSTRUCTOR_H
