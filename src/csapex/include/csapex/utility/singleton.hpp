#ifndef SINGLETON_HPP
#define SINGLETON_HPP

/// SYSTEM
#include <boost/noncopyable.hpp>

template <class T>
class Singleton : public boost::noncopyable
{
public:
    virtual ~Singleton() {

    }

    static T& instance() {
        static T inst;
        return inst;
    }
};

#endif // SINGLETON_HPP
