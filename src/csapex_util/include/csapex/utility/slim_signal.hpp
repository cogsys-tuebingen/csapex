#ifndef SLIM_SIGNAL_HPP
#define SLIM_SIGNAL_HPP

/// PROJECT
#include <csapex/utility/slim_signal.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/pack_traits.hpp>

/// SYSTEM
#include <algorithm>

namespace csapex
{
namespace slim_signal
{
template <typename Signature>
Signal<Signature>::Signal() : dirty_(false)
{
    children_.reserve(4);
}

template <typename Signature>
Signal<Signature>::~Signal()
{
    apex_assert_hard(guard_ == -1);
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    clear();
}

template <typename Signature>
Connection Signal<Signature>::connect(const delegate::Delegate<Signature>& delegate)
{
    apex_assert_hard(guard_ == -1);

    if (execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        int id = next_del_id_++;
        delegates_.emplace(id, delegate);
        execution_mutex_.unlock();

        onConnect();

        return Connection(this, makeDelegateDeleter(this, id));

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        int id = next_del_id_++;
        delegates_to_add_.emplace(id, delegate);
        dirty_ = true;

        return Connection(this, makeDelegateDeleter(this, id));
    }
}
template <typename Signature>
Connection Signal<Signature>::connect(delegate::Delegate<Signature>&& delegate)
{
    apex_assert_hard(guard_ == -1);

    if (execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        int id = next_del_id_++;
        delegates_.emplace(id, std::move(delegate));
        execution_mutex_.unlock();

        onConnect();

        return Connection(this, makeDelegateDeleter(this, id));
    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        int id = next_del_id_++;
        delegates_to_add_.emplace(id, std::move(delegate));
        dirty_ = true;

        return Connection(this, makeDelegateDeleter(this, id));
    }
}

template <typename Signature>
Connection Signal<Signature>::connect(const std::function<Signature>& fn)
{
    apex_assert_hard(guard_ == -1);

    if (execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        int id = next_fn_id_++;
        functions_.emplace(id, fn);
        execution_mutex_.unlock();

        onConnect();

        return Connection(this, makeFunctionDeleter(this, id));

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        int id = next_fn_id_++;
        functions_to_add_.emplace(id, fn);
        dirty_ = true;
        return Connection(this, makeFunctionDeleter(this, id));
    }
}

template <typename Signature>
Connection Signal<Signature>::connect(Signal<Signature>& signal)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(signal.guard_ == -1);
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    addChild(&signal);
    return Connection(this, makeSignalDeleter(this, &signal), &signal);
}

template <typename Signature>
bool Signal<Signature>::isConnected() const
{
    if (SignalBase::isConnected()) {
        return true;
    }

    return countAllConnections() > 0;
}

template <typename Signature>
int Signal<Signature>::countAllConnections() const
{
    return functions_.size() + delegates_.size() + children_.size();
}

template <typename Signature>
void Signal<Signature>::removeParent(Signal<Signature>* parent)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(parent != nullptr);
    apex_assert_hard(parent->guard_ == -1);

    std::unique_lock<std::recursive_mutex> lock(mutex_);

    for (auto it = parents_.begin(); it != parents_.end();) {
        Signal<Signature>* c = *it;
        apex_assert_hard(c->guard_ == -1);
        if (c == parent) {
            it = parents_.erase(it);
            c->removeChild(this);

        } else {
            ++it;
        }
    }
}

template <typename Signature>
void Signal<Signature>::removeDelegate(int id)
{
    apex_assert_hard(guard_ == -1);

    if (execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        delegates_.erase(id);
        execution_mutex_.unlock();

        onDisconnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        delegates_to_remove_.push_back(id);
        dirty_ = true;
    }
}

template <typename Signature>
void Signal<Signature>::removeFunction(int id)
{
    apex_assert_hard(guard_ == -1);

    if (execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        functions_.erase(id);
        execution_mutex_.unlock();

        onDisconnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        functions_to_remove_.push_back(id);
        dirty_ = true;
    }
}

template <typename Signature>
void Signal<Signature>::disconnectAll()
{
    apex_assert_hard(guard_ == -1);
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    SignalBase::disconnectAll();

    clear();
}

template <typename Signature>
void Signal<Signature>::clear()
{
    while (!parents_.empty()) {
        removeParent(parents_.front());
    }

    if (execution_mutex_.try_lock()) {
        while (!children_to_remove_.empty()) {
            removeChild(children_to_remove_.front());
        }
        while (!children_.empty()) {
            removeChild(children_.front());
        }
        execution_mutex_.unlock();

    } else {
        children_to_remove_.clear();
        children_.clear();
    }

    onDisconnect();

    functions_.clear();
    functions_to_remove_.clear();

    dirty_ = false;
}

template <typename Signature>
void Signal<Signature>::addChild(Signal* child)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(child->guard_ == -1);

    if (execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        children_.push_back(child);
        child->parents_.push_back(this);
        execution_mutex_.unlock();

        onConnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        children_to_add_.push_back(child);
        dirty_ = true;
    }
}
template <typename Signature>
void Signal<Signature>::removeChild(Signal<Signature>* child)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(child != nullptr);

    if (execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        for (auto it = children_.begin(); it != children_.end();) {
            Signal<Signature>* child_it = *it;
            // if the child exists, it has to be valid, otherwise the pointer may point to a destructed object
            apex_assert_hard(child_it->guard_ == -1);
            if (child_it == child) {
                std::vector<Connection*> to_remove;
                for (Connection* connection : connections_) {
                    if (connection->getChild() == child) {
                        to_remove.push_back(connection);
                    }
                }
                for (Connection* connection : to_remove) {
                    connection->detach();
                }
                it = children_.erase(it);
                child_it->removeParent(this);
            } else {
                ++it;
            }
        }

        execution_mutex_.unlock();

        onDisconnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        children_to_remove_.push_back(child);
        dirty_ = true;
    }
}

template <typename Signature>
template <typename... Args>
Signal<Signature>& Signal<Signature>::operator()(Args&&... args)
{
    std::lock(execution_mutex_, mutex_);

    std::unique_lock<std::recursive_mutex> exec_lock(execution_mutex_, std::adopt_lock);

    std::unique_lock<std::recursive_mutex> data_lock(mutex_, std::adopt_lock);

    for (auto& s : children_) {
        try {
            (*s)(std::forward<Args>(args)...);
        } catch (const std::exception& e) {
            printf("signal forwarding has thrown an error: %s\n", e.what());
        } catch (const csapex::Failure& e) {
            printf("signal processing function has thrown a failure: %s\n", e.what().c_str());
            throw e;
        } catch (...) {
            printf("signal forwarding has thrown an unknown error\n");
            throw;
        }
    }
    for (auto& callback : delegates_) {
        try {
            callback.second(std::forward<Args>(args)...);
        } catch (const std::exception& e) {
            printf("signal processing delegate has thrown an error: %s\n", e.what());
        } catch (const csapex::Failure& e) {
            printf("signal processing function has thrown a failure: %s\n", e.what().c_str());
            throw e;
        } catch (...) {
            printf("signal processing delegate has thrown an unknown error\n");
            throw;
        }
    }
    for (auto& fn : functions_) {
        try {
            fn.second(std::forward<Args>(args)...);
        } catch (const std::exception& e) {
            printf("signal processing function has thrown an error: %s\n", e.what());
        } catch (const csapex::Failure& e) {
            printf("signal processing function has thrown a failure: %s\n", e.what().c_str());
            throw e;
        } catch (...) {
            printf("signal processing function has thrown an unknown error\n");
            throw;
        }
    }

    data_lock.unlock();

    applyModifications();

    return *this;
}

template <typename Signature>
void Signal<Signature>::applyModifications()
{
    if (!dirty_) {
        return;
    }

    std::unique_lock<std::recursive_mutex> lock(mutex_);

    // SIGNALS
    if (!children_to_add_.empty()) {
        for (auto& s : children_to_add_) {
            children_.push_back(s);
            s->parents_.push_back(this);

            lock.unlock();
            onConnect();
            lock.lock();
        }
        children_to_add_.clear();
    }

    if (!children_to_remove_.empty()) {
        for (auto& child : children_to_remove_) {
            for (auto it = children_.begin(); it != children_.end();) {
                if (*it == child) {
                    it = children_.erase(it);

                    lock.unlock();
                    onDisconnect();
                    lock.lock();
                } else {
                    ++it;
                }
            }
        }
        children_to_remove_.clear();
    }

    // FUNCTIONS
    if (!functions_to_add_.empty()) {
        for (auto& s : functions_to_add_) {
            functions_[s.first] = std::move(s.second);

            lock.unlock();
            onConnect();
            lock.lock();
        }
        functions_to_add_.clear();
    }

    if (!functions_to_remove_.empty()) {
        for (int id : functions_to_remove_) {
            functions_.erase(id);

            lock.unlock();
            onDisconnect();
            lock.lock();
        }
        functions_to_remove_.clear();
    }

    // DELEGATES
    if (!delegates_to_add_.empty()) {
        for (auto& s : delegates_to_add_) {
            delegates_.emplace(s);

            lock.unlock();
            onConnect();
            lock.lock();
        }
        delegates_to_add_.clear();
    }

    if (!delegates_to_remove_.empty()) {
        for (int id : delegates_to_remove_) {
            delegates_.erase(id);

            lock.unlock();
            onDisconnect();
            lock.lock();
        }
        delegates_to_remove_.clear();
    }

    dirty_ = false;
}

/**
 * @brief Helper class
 */
namespace detail
{
template <typename Signature, std::size_t pos, std::size_t max, typename... AdditionalParameters>
struct MatchingConnector
{
    template <typename ActualResult, typename... ActualParameters>
    static Connection connect(Signal<Signature>* signal, const std::function<ActualResult(ActualParameters...)>& fn)
    {
        const std::size_t end = csapex::function_traits<Signature>::arity;
        using Type = typename csapex::function_traits<Signature>::template arg<end - pos - 1>::type;
        return MatchingConnector<Signature, pos + 1, max - 1, Type, AdditionalParameters...>::connect(signal, fn);
    }

    template <typename ActualResult, typename... ActualParameters>
    static Connection connect(Signal<Signature>* signal, const Signal<ActualResult(ActualParameters...)>& fn)
    {
        const std::size_t end = csapex::function_traits<Signature>::arity;
        using Type = typename csapex::function_traits<Signature>::template arg<end - pos - 1>::type;
        return MatchingConnector<Signature, pos + 1, max - 1, Type, AdditionalParameters...>::connect(signal, fn);
    }
};

template <typename Signature, std::size_t pos, typename... AdditionalParameters>
struct MatchingConnector<Signature, pos, 0, AdditionalParameters...>
{
    template <typename ActualResult, typename... ActualParameters>
    static Connection connect(Signal<Signature>* signal, const std::function<ActualResult(ActualParameters...)>& fn,
                              typename std::enable_if<sizeof...(ActualParameters) == function_traits<Signature>::arity, int>::type* = 0)
    {
        return signal->connect(static_cast<std::function<Signature>>(fn));
    }

    template <typename ActualResult, typename... ActualParameters>
    static Connection connect(Signal<Signature>* signal, const std::function<ActualResult(ActualParameters...)>& fn,
                              typename std::enable_if<sizeof...(ActualParameters) < function_traits<Signature>::arity, int>::type* = 0)
    {
        using ReturnType = typename csapex::function_traits<Signature>::result_type;
        std::function<Signature> callback = [fn](ActualParameters... params, AdditionalParameters... /*drop*/) -> ReturnType { fn(params...); };
        return signal->connect(callback);
    }

    template <typename ActualResult, typename... ActualParameters>
    static Connection connect(Signal<Signature>* signal, Signal<ActualResult(ActualParameters...)>& sig,
                              typename std::enable_if<is_signature_equal<Signature, ActualResult, ActualParameters...>::value, int>::type* = 0)
    {
        static_assert(is_signature_equal<Signature, ActualResult, ActualParameters...>::value, "Signals match");
        return signal->connect(sig);
    }

    template <typename ActualResult, typename... ActualParameters>
    static Connection connect(Signal<Signature>* signal, Signal<ActualResult(ActualParameters...)>& sig,
                              typename std::enable_if<!is_signature_equal<Signature, ActualResult, ActualParameters...>::value, int>::type* = 0)
    {
        static_assert(!is_signature_equal<Signature, ActualResult, ActualParameters...>::value, "Signals don't match");

        using ReturnType = typename csapex::function_traits<Signature>::result_type;
        std::function<Signature> callback = [&sig](ActualParameters... params, AdditionalParameters... /*drop*/) -> ReturnType { sig(params...); };
        return signal->connect(callback);
    }
};
}  // namespace detail

template <typename Signature>
template <typename Callable>
Connection Signal<Signature>::connectAny(Callable& function, tag_is_function<false, false>)
{
    static_assert(!std::is_base_of<SignalBase, Callable>::value, "SignalBase assert");
    static_assert(!std::is_bind_expression<Callable>::value, "Bind assert");

    using FunctionType = typename std::decay<decltype(function)>::type;
    const std::size_t available = csapex::function_traits<Signature>::arity;
    const std::size_t needed = csapex::function_traits<FunctionType>::arity;

    std::function<typename function_traits<FunctionType>::signature> fn = function;
    return std::move(detail::MatchingConnector<Signature, 0, available - needed>::connect(this, fn));
}

template <typename Signature>
template <typename Callable>
Connection Signal<Signature>::connectAny(Callable& function, tag_is_function<false, true>)
{
    std::function<Signature> fn = function;
    return std::move(connect(fn));
}

template <typename Signature>
template <typename Callable>
Connection Signal<Signature>::connectAny(Callable& signal, tag_is_function<true, false>)
{
    static_assert(std::is_base_of<SignalBase, Callable>::value, "SignalBase assert");
    static_assert(!std::is_bind_expression<Callable>::value, "Bind assert");

    using FunctionType = typename Callable::signature_t;
    const std::size_t available = csapex::function_traits<Signature>::arity;
    const std::size_t needed = csapex::function_traits<FunctionType>::arity;

    return std::move(detail::MatchingConnector<Signature, 0, available - needed>::connect(this, signal));
}

template <typename Signature>
Connection::Deleter Signal<Signature>::makeFunctionDeleter(Signal<Signature>* parent, int id)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(parent->guard_ == -1);
    return [parent, id]() {
        apex_assert_hard(parent->guard_ == -1);
        parent->removeFunction(id);
    };
}

template <typename Signature>
Connection::Deleter Signal<Signature>::makeDelegateDeleter(Signal<Signature>* parent, int id)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(parent->guard_ == -1);
    return [parent, id] {
        apex_assert_hard(parent->guard_ == -1);
        parent->removeDelegate(id);
    };
}

template <typename Signature>
Connection::Deleter Signal<Signature>::makeSignalDeleter(Signal<Signature>* parent, Signal<Signature>* sig)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(parent->guard_ == -1);
    apex_assert_hard(sig->guard_ == -1);
    return [parent, sig] {
        apex_assert_hard(parent->guard_ == -1);
        apex_assert_hard(sig->guard_ == -1);
        parent->removeChild(sig);
    };
}

template <typename Signature>
void ObservableSignal<Signature>::onConnect()
{
    if (Signal<Signature>::countAllConnections() == 1) {
        first_connected();
    }

    connected();
}

template <typename Signature>
void ObservableSignal<Signature>::onDisconnect()
{
    disconnected();

    if (Signal<Signature>::countAllConnections() == 0) {
        last_disconnected();
    }
}

}  // namespace slim_signal
}  // namespace csapex

#endif  // SLIM_SIGNAL_HPP
