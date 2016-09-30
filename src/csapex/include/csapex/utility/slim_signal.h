#ifndef SLIM_SIGNAL_H
#define SLIM_SIGNAL_H

/// PROJECT
#include <csapex/csapex_util_export.h>
#include <csapex/utility/delegate.h>

/// SYSTEM
#include <vector>
#include <map>
#include <functional>
#include <mutex>

namespace csapex
{
namespace slim_signal
{
/// FORWARD DECLARATION
class Connection;
class ScopedConnection;

/**
 * @brief The SignalBase class is the inferface for signals
 */
class CSAPEX_UTILS_EXPORT SignalBase
{
    friend class Connection;
    friend class ScopedConnection;

public:
    virtual ~SignalBase();

    virtual void disconnectAll();

protected:
    SignalBase();

    SignalBase(const SignalBase&) = delete;
    SignalBase& operator= (const SignalBase&) = delete;

    void addConnection(Connection* connection);
    void removeConnection(const Connection* connection);

protected:
    mutable std::recursive_mutex mutex_;

private:
    std::vector<Connection*> connections_;

protected:
    long guard_ = -1;
};



/**
 * @brief The Connection class is a handle for a signal connection
 */
class CSAPEX_UTILS_EXPORT Connection
{
    friend class ScopedConnection;
    friend class SignalBase;

public:
    typedef std::function<void()> Deleter;

    Connection(SignalBase* parent, const Deleter& del);
    Connection(const Connection& c);
    Connection();

    virtual ~Connection();

    void detach() const;
    void disconnect() const;

private:
    mutable SignalBase* parent_;
    mutable bool detached_ = false;
    Deleter deleter_;
};



/**
 * @brief The ScopedConnection class is a handle for a signal connection
 * that automatically disconnects on deletion
 *
 * A scoped connection can't be copied, only moved.
 * If a scoped connection is moved from, the signal will not be disconnected.
 */
class CSAPEX_UTILS_EXPORT ScopedConnection : public Connection
{
public:
    ScopedConnection();
    ScopedConnection(const Connection& c);
    ScopedConnection(ScopedConnection&& c) noexcept;
    ScopedConnection(const ScopedConnection& c) = delete;

    ~ScopedConnection();

    void operator = (const Connection& c);
    void operator = (const ScopedConnection& c) = delete;
    void operator = (ScopedConnection&& c) noexcept;
};

/**
 * @brief The Signal template is the class implementing a signal for a specific function type
 */
template <typename Signature>
class Signal : public SignalBase
{
public:
    Signal();

    Signal(const Signal&) = delete;
    Signal& operator= (const Signal&) = delete;

    ~Signal();

    Connection connect(const delegate::Delegate<Signature>& delegate);
    Connection connect(delegate::Delegate<Signature>&& delegate);
    Connection connect(const std::function<Signature>& fn);

    Connection connect(Signal<Signature>& signal);

    template <typename Class>
    Connection connect(Class* that, Signature Class::*mem) {
        return connect(std::move(delegate::Delegate<Signature>(that, mem)));
    }

    template <typename... Args>
    Signal& operator () (Args... args);

    virtual void disconnectAll() override;

private:
    void clear();
    void removeParent(Signal* parent);

    void addChild(Signal* child);
    void removeChild(Signal* child);

    void removeDelegate(int id);
    void removeFunction(int id);

    void applyModifications();

private:
    Connection::Deleter makeFunctionDeleter(Signal<Signature>* parent, int id);
    Connection::Deleter makeDelegateDeleter(Signal<Signature>* parent, int id);
    Connection::Deleter makeSignalDeleter(Signal<Signature>* parent, Signal<Signature>* sig);

private:
    mutable std::recursive_mutex execution_mutex_;

    int next_del_id_ = 0;
    std::map<int, delegate::Delegate<Signature>> delegates_;
    std::map<int, delegate::Delegate<Signature>> delegates_to_add_;
    std::vector<int> delegates_to_remove_;

    int next_fn_id_ = 0;
    std::map<int, std::function<Signature>> functions_;
    std::map<int, std::function<Signature>> functions_to_add_;
    std::vector<int> functions_to_remove_;

    std::vector<Signal<Signature>*> children_;
    std::vector<Signal<Signature>*> children_to_add_;
    std::vector<Signal<Signature>*> children_to_remove_;

    std::vector<Signal<Signature>*> parents_;
};

}
}

#endif // SLIM_SIGNAL_H
