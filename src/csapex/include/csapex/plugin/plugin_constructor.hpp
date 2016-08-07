#ifndef PLUGIN_CONSTRUCTOR_HPP
#define PLUGIN_CONSTRUCTOR_HPP

/// COMPONENT
#include <csapex/utility/constructor.hpp>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{
template <class M>
struct PluginConstructor : public ConstructorInterface {

public:
    typedef std::function<typename std::shared_ptr<M>()> Call;


public:
    PluginConstructor()
    {
    }

    typename std::shared_ptr<M> operator()() const {
        return construct();
    }

    std::shared_ptr<M> construct() const {
        std::shared_ptr<M> res(constructor());
//        impl::setType<M>(res, type);
        if(!res) {
            throw std::runtime_error(std::string("cannot construct class ") + type);
        }
        instances_.push_back(res);
        return res;
    }

    bool valid() const override {
        typename std::shared_ptr<M> res(constructor());
        return res.get() != nullptr;
    }

    void setConstructor(Call c) {
        constructor = c;
        has_constructor = true;
    }

    void setLibraryName(const std::string& library_name) {
        library_name_ = library_name;
    }

    std::string getLibraryName() const {
        return library_name_;
    }

    std::vector< std::weak_ptr<M> > getInstances() {
        return instances_;
    }

private:
    Call constructor;

    std::string library_name_;
    mutable std::vector< std::weak_ptr<M> > instances_;
};
}

#endif // PLUGIN_CONSTRUCTOR_HPP

