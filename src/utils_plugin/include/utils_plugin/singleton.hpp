#ifndef PLUGIN_HPP
#define PLUGIN_HPP

/// SYSTEM
#include <boost/noncopyable.hpp>

template <class T>
class Singleton : public boost::noncopyable
{
public:
    static T& instance() {
        static T inst;
        return inst;
    }
};

#endif // PLUGIN_HPP
