#ifndef SINGLETON_HPP
#define SINGLETON_HPP

/// SYSTEM
#include <vector>
#include <mutex>

namespace csapex
{

class SingletonInterface
{
public:
    static void shutdownAll();

    virtual void shutdown();

protected:
    SingletonInterface();

private:
    SingletonInterface(const SingletonInterface& other) = delete;
    SingletonInterface(SingletonInterface&& other) = delete;
    SingletonInterface& operator =(const SingletonInterface& other) = delete;

private:
    static std::mutex instances_mutex_;
    static std::vector<SingletonInterface*> instances_;
};

template <class T>
class Singleton : public SingletonInterface
{
public:
    virtual ~Singleton() {

    }

    static T& instance() {
        static T inst;
        return inst;
    }
};

}

#endif // SINGLETON_HPP
