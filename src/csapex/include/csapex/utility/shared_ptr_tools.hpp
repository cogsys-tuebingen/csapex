#ifndef SHARED_PTR_TOOLS_HPP
#define SHARED_PTR_TOOLS_HPP

namespace shared_ptr_tools
{
//template<typename T>
//std::shared_ptr<T> to_std_shared(boost::shared_ptr<T>& ptr)
//{
//    return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
//}
//template<typename T>
//std::shared_ptr<T> to_std_shared(const std::shared_ptr<T>& ptr)
//{
//    return ptr;
//}
//template<typename T>
//boost::shared_ptr<T> to_boost_shared(std::shared_ptr<T>& ptr)
//{
//    return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
//}
//template<typename T>
//boost::shared_ptr<T> to_boost_shared(const boost::shared_ptr<T>& ptr)
//{
//    return ptr;
//}

namespace detail
{
template<class SharedPointer> struct Holder
{
    SharedPointer p;

    Holder(const SharedPointer &p) : p(p) {}
    Holder(const Holder &other) : p(other.p) {}
    Holder(Holder &&other) : p(std::move(other.p)) {}

    void operator () (...) { p.reset(); }
};
}

template<class T> std::shared_ptr<T> to_std_shared(const boost::shared_ptr<T> &p) {
    typedef detail::Holder<std::shared_ptr<T>> H;
    if(H *h = boost::get_deleter<H, T>(p)) {
        return h->p;
    } else {
        return std::shared_ptr<T>(p.get(), detail::Holder<boost::shared_ptr<T>>(p));
    }
}

template<class T> boost::shared_ptr<T> to_boost_shared(const std::shared_ptr<T> &p){
    typedef detail::Holder<boost::shared_ptr<T>> H;
    if(H * h = std::get_deleter<H, T>(p)) {
        return h->p;
    } else {
        return boost::shared_ptr<T>(p.get(), detail::Holder<std::shared_ptr<T>>(p));
    }
}

}


#endif // SHARED_PTR_TOOLS_HPP

