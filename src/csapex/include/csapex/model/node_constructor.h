#ifndef NODE_CONSTRUCTOR_H
#define NODE_CONSTRUCTOR_H

/// COMPONENT
#include <utils_plugin/constructor.hpp>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

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
    typedef boost::function<NodePtr()> Make;

    typedef boost::shared_ptr<NodeConstructor> Ptr;

    static NodePtr makeNull();

public:
    NodeConstructor(const std::string& type, const std::string& description, Make c);

    virtual ~NodeConstructor();

    std::string getType() const;
    std::vector<Tag> getTags() const;
    QIcon getIcon() const;
    std::string getDescription() const;

    virtual NodePtr makePrototypeContent() const;
    virtual NodePtr makeContent(const UUID& uuid) const;

protected:
    NodeConstructor(const std::string& type, const std::string& description);

    virtual void load() const;

protected:
    std::string type_;
    std::string descr_;

    mutable bool is_loaded;
    mutable QIcon icon;
    mutable std::vector<Tag> cat;

    Make c;
};

}
#endif // NODE_CONSTRUCTOR_H
