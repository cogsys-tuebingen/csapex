#ifndef NODE_CONSTRUCTOR_H
#define NODE_CONSTRUCTOR_H

/// COMPONENT
#include <csapex/utility/constructor.hpp>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>
#include <utils_param/param_fwd.h>

/// SYSTEM
#include <typeinfo>
#include <boost/function.hpp>
#include <QIcon>

namespace csapex
{

class NodeConstructor
{
    friend class command::AddNode;
    friend class BoxManager;

public:
    struct NodeConstructionException : public std::runtime_error
    {
        NodeConstructionException(const std::string& what)
            : std::runtime_error(what)
        {
        }
    };

public:
    typedef boost::function<NodePtr()> Make;

    typedef boost::shared_ptr<NodeConstructor> Ptr;

    static NodePtr makeNull();

public:
    NodeConstructor(Settings& settings, const std::string& type,
                    const std::string& description, const std::string& icon,
                    Make c);

    virtual ~NodeConstructor();

    std::string getType() const;
    std::vector<TagPtr> getTags() const;
    QIcon getIcon() const;
    std::string getDescription() const;

    virtual NodePtr makePrototypeContent() const;
    virtual NodePtr makeContent(const UUID& uuid) const;

protected:
    NodeConstructor(Settings &settings, const std::string& type, const std::string& description, const std::string &icon);

    virtual void load() const;

protected:
    Settings& settings_;
    std::string type_;
    std::string descr_;
    std::string icon_;

    mutable bool is_loaded;
    mutable std::vector<TagPtr> tags_;

    Make c;
};

}
#endif // NODE_CONSTRUCTOR_H
