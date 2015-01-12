#ifndef PLUGIN_CONSTRUCTOR_HPP
#define PLUGIN_CONSTRUCTOR_HPP

/// COMPONENT
#include <csapex/utility/constructor.hpp>

/// SYSTEM
#include <boost/signals2.hpp>

namespace csapex
{
template <class M>
struct PluginConstructor : public ConstructorInterface {

public:
    typedef boost::function<typename boost::shared_ptr<M>()> Call;

public:
    boost::shared_ptr< boost::signals2::signal<void()> > unload_request;
    boost::shared_ptr< boost::signals2::signal<void()> > reload_request;

public:
    PluginConstructor()
        : unload_request(new boost::signals2::signal<void()>),
          reload_request(new boost::signals2::signal<void()>)
    {

    }

    typename boost::shared_ptr<M> operator()() const {
        return construct();
    }

    boost::shared_ptr<M> construct() const {
        boost::shared_ptr<M> res(constructor());
//        impl::setType<M>(res, type);
        assert(res.get() != nullptr);
        instances_.push_back(res);
        return res;
    }

    bool valid() const {
        typename boost::shared_ptr<M> res(constructor());
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

    std::vector< boost::weak_ptr<M> > getInstances() {
        return instances_;
    }

    void unload() {
        (*unload_request)();
    }

    void reload() {
        (*reload_request)();
    }

private:
    Call constructor;

    std::string library_name_;
    mutable std::vector< boost::weak_ptr<M> > instances_;
};
}

#endif // PLUGIN_CONSTRUCTOR_HPP

