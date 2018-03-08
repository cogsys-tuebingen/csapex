#ifndef SLIM_SIGNAL_H
#define SLIM_SIGNAL_H

/// PROJECT
#include <csapex_util_export.h>
#include <csapex/utility/delegate.h>
#include <csapex/utility/function_traits.hpp>

/// SYSTEM
#include <vector>
#include <map>
#include <functional>
#include <mutex>
#include <type_traits>
#include <atomic>

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
    virtual bool isConnected() const;

protected:
    SignalBase();

    SignalBase(const SignalBase&) = delete;
    SignalBase& operator= (const SignalBase&) = delete;

    void addConnection(Connection* connection);
    void removeConnection(const Connection* connection);

    virtual int countAllConnections() const = 0;

protected:
    virtual void onConnect();
    virtual void onDisconnect();

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
    using signature_t = Signature;

public:
    Signal();

    Signal(const Signal&) = delete;
    Signal& operator= (const Signal&) = delete;

    ~Signal();

    // connections with matching signatures
    Connection connect(const delegate::Delegate<Signature>& delegate);
    Connection connect(delegate::Delegate<Signature>&& delegate);
    Connection connect(const std::function<Signature>& fn);

    Connection connect(Signal<Signature> &signal);

    template <typename Class>
    Connection connect(Class* that, Signature Class::*mem) {
        return connect(std::move(delegate::Delegate<Signature>(that, mem)));
    }

    // generic connections with fewer parameters than available
    template <typename Callable>
    Connection connect(Callable function,
                       typename std::enable_if<!std::is_base_of<SignalBase, Callable>::value>::type* = 0)
    {
        return connectAny(function, tag_is_function<
                          std::is_base_of<SignalBase, Callable>::value,
                          std::is_bind_expression<Callable>::value
                          >());
    }

    template <typename MatchingSignature>
    Connection connect(Signal<MatchingSignature>& signal,
                       typename std::enable_if<!std::is_same<Signature, MatchingSignature>::value>::type* = 0)
    {
        return connectAny(signal, tag_is_function<true, false>());
    }

    virtual bool isConnected() const override;
    virtual int countAllConnections() const override;

    template <typename... Args>
    Signal& operator () (Args&&... args);

    virtual void disconnectAll() override;

private:
    template <bool signal, bool bind>
    struct tag_is_function {};

    template <typename Callable>
    Connection connectAny(Callable& function, tag_is_function<false, false>);

    template <typename Callable>
    Connection connectAny(Callable& function, tag_is_function<true, false>);

    template <typename Callable>
    Connection connectAny(Callable& function, tag_is_function<false, true>);

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

    std::atomic<bool> dirty_;

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

/**
 * @brief The ObservableSignal template implements a signal that emits special signals itself
 */
template <typename Signature>
class ObservableSignal : public Signal<Signature>
{
public:
    Signal<void()> connected;
    Signal<void()> disconnected;

    Signal<void()> first_connected;
    Signal<void()> last_disconnected;

private:
    virtual void onConnect() override;
    virtual void onDisconnect() override;
};

}

}

#endif // SLIM_SIGNAL_H
