#ifndef SLIM_SIGNAL_HPP
#define SLIM_SIGNAL_HPP

/// PROJECT
#include <csapex/utility/slim_signal.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <algorithm>

namespace csapex
{
namespace slim_signal
{

template <typename Signature>
Signal<Signature>::Signal()
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

    if(execution_mutex_.try_lock()) {
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

        return Connection(this, makeDelegateDeleter(this, id));
    }

}
template <typename Signature>
Connection Signal<Signature>::connect(delegate::Delegate<Signature>&& delegate)
{
    apex_assert_hard(guard_ == -1);

    if(execution_mutex_.try_lock()) {
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

        return Connection(this, makeDelegateDeleter(this, id));
    }
}

template <typename Signature>
Connection Signal<Signature>::connect(const std::function<Signature>& fn)
{
    apex_assert_hard(guard_ == -1);

    if(execution_mutex_.try_lock()) {
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
    return Connection(this, makeSignalDeleter(this, &signal));
}

template <typename Signature>
bool Signal<Signature>::isConnected() const
{
    if(SignalBase::isConnected()) {
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

    for(auto it = parents_.begin(); it != parents_.end();) {
        Signal<Signature>* c = *it;
        apex_assert_hard(c->guard_ == -1);
        if(c == parent) {
            it =  parents_.erase(it);
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

    if(execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        delegates_.erase(id);
        execution_mutex_.unlock();

        onDisconnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        delegates_to_remove_.push_back(id);
    }
}


template <typename Signature>
void Signal<Signature>::removeFunction(int id)
{
    apex_assert_hard(guard_ == -1);

    if(execution_mutex_.try_lock()) {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        functions_.erase(id);
        execution_mutex_.unlock();

        onDisconnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        functions_to_remove_.push_back(id);
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
    while(!parents_.empty()) {
        removeParent(parents_.front());
    }

    if(execution_mutex_.try_lock()) {
        while(!children_to_remove_.empty()) {
            removeChild(children_to_remove_.front());
        }
        while(!children_.empty()) {
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
}

template <typename Signature>
void Signal<Signature>::addChild(Signal *child)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(child->guard_ == -1);

    if(execution_mutex_.try_lock()) {

        std::unique_lock<std::recursive_mutex> lock(mutex_);
        children_.push_back(child);
        child->parents_.push_back(this);
        execution_mutex_.unlock();

        onConnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        children_to_add_.push_back(child);
    }
}
template <typename Signature>
void Signal<Signature>::removeChild(Signal<Signature>* child)
{
    apex_assert_hard(guard_ == -1);
    apex_assert_hard(child != nullptr);

    if(execution_mutex_.try_lock()) {

        std::unique_lock<std::recursive_mutex> lock(mutex_);
        for(auto it = children_.begin(); it != children_.end();) {
            Signal<Signature>* c = *it;
            // if the child exists, it has to be valid, otherwise the pointer may point to a destructed object
            apex_assert_hard(c->guard_ == -1);
            if(c == child) {
                it =  children_.erase(it);
                c->removeParent(this);
            } else {
                ++it;
            }
        }

        execution_mutex_.unlock();

        onDisconnect();

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        children_to_remove_.push_back(child);
    }
}



template <typename Signature>
template <typename... Args>
Signal<Signature>& Signal<Signature>::operator () (Args&&... args)
{
    apex_assert_hard(guard_ == -1);

    std::unique_lock<std::recursive_mutex> lock(execution_mutex_);

    for(auto& s : children_) {
        apex_assert_hard(s->guard_ == -1);
        (*s)(std::forward<Args>(args)...);
    }
    for(auto& callback : delegates_) {
        callback.second(std::forward<Args>(args)...);
    }
    for(auto& fn : functions_) {
        fn.second(std::forward<Args>(args)...);
    }

    applyModifications();

    return *this;
}

template <typename Signature>
void Signal<Signature>::applyModifications()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    // SIGNALS
    for(auto& s : children_to_add_) {
        children_.push_back(s);
        s->parents_.push_back(this);

        lock.unlock();
        onConnect();
        lock.lock();
    }
    children_to_add_.clear();

    for(auto& child : children_to_remove_) {
        for(auto it = children_.begin(); it != children_.end();) {
            if(*it == child) {
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

    // FUNCTIONS
    for(auto& s : functions_to_add_) {
        functions_[s.first] = std::move(s.second);

        lock.unlock();
        onConnect();
        lock.lock();
    }
    functions_to_add_.clear();

    for(int id : functions_to_remove_) {
        functions_.erase(id);

        lock.unlock();
        onDisconnect();
        lock.lock();
    }
    functions_to_remove_.clear();

    // DELEGATES
    for(auto& s : delegates_to_add_) {
        delegates_.emplace(s);

        lock.unlock();
        onConnect();
        lock.lock();
    }
    delegates_to_add_.clear();

    for(int id : delegates_to_remove_) {
        delegates_.erase(id);

        lock.unlock();
        onDisconnect();
        lock.lock();
    }
    delegates_to_remove_.clear();
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
Connection::Deleter Signal<Signature>::makeDelegateDeleter(Signal<Signature>* parent,int id)
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
        parent->removeChild(sig);
    };
}

template <typename Signature>
void Signal<Signature>::onConnect()
{
}

template <typename Signature>
void Signal<Signature>::onDisconnect()
{
}




template <typename Signature>
void ObservableSignal<Signature>::onConnect()
{
    if(Signal<Signature>::countAllConnections() == 1) {
        first_connected();
    }

    connected();
}

template <typename Signature>
void ObservableSignal<Signature>::onDisconnect()
{
    disconnected();

    if(Signal<Signature>::countAllConnections() == 0)  {
        last_disconnected();
    }
}

}
}

#endif // SLIM_SIGNAL_HPP

