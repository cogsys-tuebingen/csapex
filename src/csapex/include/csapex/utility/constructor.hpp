#ifndef CONSTRUCTOR_HPP
#define CONSTRUCTOR_HPP

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

struct Constructor {
    Constructor() : valid_(false), has_constructor(false) {}

    virtual bool valid() const {
        return valid_ && has_constructor;
    }

    std::string getType() const {
        return type;
    }

    void setType(const std::string& n) {
        type = n;
    }

    std::string getDescription() const {
        return descr;
    }

    void setDescription(const std::string& n) {
        descr = n;
    }

    std::string getTags() const {
        return tags;
    }

    void setTags(const std::string& t) {
        tags = t;
    }

    std::string getIcon() const {
        return icon;
    }

    void setIcon(const std::string& i) {
        icon = i;
    }

protected:
    std::string type;
    std::string descr;
    std::string icon;
    std::string tags;
    bool valid_;
    bool has_constructor;
};



template <typename Any>
class HasName
{
    typedef char Small;
    class Big { char dummy[2]; };

    template <typename Class> static Small test(typeof(&Class::setName)) ;
    template <typename Class> static Big test(...);

public:
    enum { value = sizeof(test<Any>(0)) == sizeof(Small) };
};

namespace impl {
template <typename M>
typename boost::enable_if<HasName<M>, void>::type
setType(boost::shared_ptr<M> res, const std::string& type)
{
    res->setName(type);
}

template <typename M>
typename boost::disable_if<HasName<M>, void>::type
setType(boost::shared_ptr<M> res, const std::string& type)
{
    res->setType(type);
}

}

template <class M>
struct DefaultConstructor : public Constructor {

    typedef boost::function<typename boost::shared_ptr<M>()> Call;

    typename boost::shared_ptr<M> operator()() const {
        return construct();
    }

    boost::shared_ptr<M> construct() const {
        boost::shared_ptr<M> res(constructor());
        impl::setType<M>(res, type);
        assert(res.get() != NULL);
        return res;
    }

    bool valid() const {
        typename boost::shared_ptr<M> res(constructor());
        return res.get() != NULL;
    }

    void setConstructor(Call c) {
        constructor = c;
        has_constructor = true;
    }

private:
    Call constructor;
};

#endif // CONSTRUCTOR_HPP
