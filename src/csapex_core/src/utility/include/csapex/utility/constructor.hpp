#ifndef CONSTRUCTOR_HPP
#define CONSTRUCTOR_HPP

/// SYSTEM
#include <memory>
#include <functional>
#include <vector>
#include <type_traits>

struct ConstructorInterface {
    ConstructorInterface() : valid_(false), has_constructor(false) {}

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



//template <typename Any>
//class HasName
//{
//    typedef char Small;
//    class Big { char dummy[2]; };

//    template <typename Class> static Small test(typeof(&Class::setName)) ;
//    template <typename Class> static Big test(...);

//public:
//    enum { value = sizeof(test<Any>(0)) == sizeof(Small) };
//};

template<typename> struct Void { typedef void type; };

template<typename T, typename Sfinae = void>
struct HasName: std::false_type {};

template<typename T>
struct HasName<
    T
    , typename Void<
        decltype( std::declval<T&>().setName(0) )
    >::type
>: std::true_type {};


namespace impl {
template <typename M>
typename std::enable_if<HasName<M>::value, void>::type
setType(std::shared_ptr<M> res, const std::string& type)
{
    res->setName(type);
}

template <typename M>
typename std::enable_if<!HasName<M>::value, void>::type
setType(std::shared_ptr<M> res, const std::string& type)
{
    res->setType(type);
}

}

#endif // CONSTRUCTOR_HPP
