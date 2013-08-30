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

protected:
    std::string type;
    std::string descr;
    bool valid_;
    bool has_constructor;
};

template <class M>
struct DefaultConstructor : public Constructor {

    typedef boost::function<typename boost::shared_ptr<M>()> Call;

    typename boost::shared_ptr<M> operator()() const {
        return construct();
    }

    typename boost::shared_ptr<M> construct() const {
        boost::shared_ptr<M> res(constructor());
        res->setName(type);
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
