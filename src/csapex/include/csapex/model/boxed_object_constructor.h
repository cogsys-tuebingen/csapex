#ifndef BOXED_OBJECT_CONSTRUCTOR_H
#define BOXED_OBJECT_CONSTRUCTOR_H

/// COMPONENT
#include <utils_plugin/constructor.hpp>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <typeinfo>
#include <boost/function.hpp>
#include <QIcon>

namespace csapex
{

class BoxedObjectConstructor
{
    friend class command::AddNode;
    friend class BoxManager;

public:
    typedef boost::function<BoxedObjectPtr()> Make;

    typedef boost::shared_ptr<BoxedObjectConstructor> Ptr;

    static BoxedObjectPtr makeNull();

public:
    BoxedObjectConstructor(const std::string& type, const std::string& description, Make c);

    virtual ~BoxedObjectConstructor();

    std::string getType() const;
    std::vector<Tag> getTags() const;
    QIcon getIcon() const;
    std::string getDescription() const;

    virtual BoxedObjectPtr makePrototypeContent() const;
    virtual BoxedObjectPtr makeContent(const std::string& uuid) const;

protected:
    BoxedObjectConstructor(const std::string& type, const std::string& description);

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
#endif // BOXED_OBJECT_CONSTRUCTOR_H
