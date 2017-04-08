#ifndef OBSERVER_H_
#define OBSERVER_H_

/// COMPONENT
#include <csapex/utility/slim_signal.h>

/// SYSTEM
#include <memory>

namespace csapex
{

class Observer
{
public:
    virtual ~Observer();

protected:
    void stopObserving();

    void manageConnection(slim_signal::ScopedConnection&& connection);
    void manageConnection(const slim_signal::Connection& connection);

    template <typename Signature, typename Lambda>
    void observe(slim_signal::Signal<Signature>& signal, Lambda callback)
    {
        manageConnection(std::move(signal.connect(callback)));
    }
    template <typename Signature, typename CompatibleSignature>
    void observe(slim_signal::Signal<Signature>& signal, slim_signal::Signal<CompatibleSignature>& compatible_signal)
    {
        manageConnection(std::move(signal.connect(compatible_signal)));
    }
    template <typename Signature, typename Lambda>
    void observe(slim_signal::Signal<Signature>* signal, Lambda callback)
    {
        manageConnection(std::move(signal->connect(callback)));
    }
    template <typename Signature, typename Lambda>
    void observe(std::shared_ptr<slim_signal::Signal<Signature>>& signal, Lambda callback)
    {
        manageConnection(std::move(signal->connect(callback)));
    }

    template <typename Signature>
    void observe(slim_signal::Signal<Signature>& signal, slim_signal::Signal<Signature>& relay)
    {
        manageConnection(std::move(signal.connect(relay)));
    }
    template <typename Signature>
    void observe(slim_signal::Signal<Signature>* signal, slim_signal::Signal<Signature>& relay)
    {
        manageConnection(std::move(signal->connect(relay)));
    }
    template <typename Signature>
    void observe(std::shared_ptr<slim_signal::Signal<Signature>>& signal, slim_signal::Signal<Signature>& relay)
    {
        manageConnection(std::move(signal->connect(relay)));
    }

private:
    std::vector<slim_signal::ScopedConnection> observed_connections_;
};

}

#endif // OBSERVER_H_
