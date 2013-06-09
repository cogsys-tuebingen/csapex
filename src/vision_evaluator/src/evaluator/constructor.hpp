#ifndef CONSTRUCTOR_HPP
#define CONSTRUCTOR_HPP

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

struct Constructor {
    std::string name;
    bool valid_;

    Constructor() : valid_(false) {}

    virtual bool valid() const {
        return valid_;
    }

    std::string getName() const {
        return name;
    }
};

template <class M>
struct DefaultConstructor : public Constructor {
    boost::function<M*()> constructor;

    typename boost::shared_ptr<M> operator()() const {
        return construct();
    }

    typename boost::shared_ptr<M> construct() const {
        boost::shared_ptr<M> res(constructor());
        res->setName(name);
        assert(res.get() != NULL);
        return res;
    }

    bool valid() const {
        typename boost::shared_ptr<M> res(constructor());
        return res.get() != NULL;
    }
};

#endif // CONSTRUCTOR_HPP
