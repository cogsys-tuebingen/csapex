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

    std::string getName() const {
        return name;
    }

    void setName(const std::string& n) {
        name = n;
    }

protected:
    std::string name;
    bool valid_;
    bool has_constructor;
};

template <class M>
struct DefaultConstructor : public Constructor {

    typedef boost::function<M*()> Call;

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

    void setConstructor(Call c) {
        constructor = c;
        has_constructor = true;
    }

private:
    Call constructor;
};

#endif // CONSTRUCTOR_HPP
