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
    friend class command::AddBox;
    friend class BoxManager;

public:
    typedef boost::function<BoxedObjectPtr()> Make;

    typedef boost::shared_ptr<BoxedObjectConstructor> Ptr;
    static const Ptr NullPtr;

public:
    struct ProxyConstructor : public Constructor {
        typedef boost::function<BoxedObjectConstructor::Ptr (const std::string)> Call;

        BoxedObjectConstructor::Ptr operator()() {
            BoxedObjectConstructor::Ptr res(constructor(type));
            assert(res != NULL);
            return res;
        }

        void setConstructor(Call c) {
            constructor = c;
            has_constructor = true;
        }

    private:
        Call constructor;
    };

public:
    BoxedObjectConstructor(const std::string& type, const std::string& description, Make c);

    virtual ~BoxedObjectConstructor();

    std::string getType() const;
    std::vector<Tag> getTags() const;
    QIcon getIcon() const;
    std::string getDescription() const;

    BoxedObjectPtr makeContent() const;

protected:
    std::string type_;
    std::string descr_;
    QIcon icon;
    std::vector<Tag> cat;

    Make c;
};

}
#endif // BOXED_OBJECT_CONSTRUCTOR_H
