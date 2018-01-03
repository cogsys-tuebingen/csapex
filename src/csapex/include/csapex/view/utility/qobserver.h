#ifndef QOBSERVER_H
#define QOBSERVER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <mutex>
#include <functional>
#include <QObject>

namespace csapex
{

#ifdef __GNUC__
#  include <features.h>
#  if __GNUC_PREREQ(4,9)
#define USE_GCC47_WORKAROUND 0
#  else
#define USE_GCC47_WORKAROUND 1
#  endif
#else
//    If not gcc
#endif

#if USE_GCC47_WORKAROUND
namespace gcc47
{

template<int ...>
struct seq { };

template<int N, int ...S>
struct generateSequence : generateSequence<N-1, N-1, S...> { };

template<int ...S>
struct generateSequence<0, S...> {
    typedef seq<S...> type;
};

template<typename Function, typename Tuple, int ...S>
void call(seq<S...>, Function F, Tuple tuple) {
    (void) tuple;
    F(std::get<S>(tuple) ...);
}

template<typename Instance, typename Function, typename Tuple, int ...S>
void call(seq<S...>, Instance* i, Function F, Tuple tuple) {
    (void) tuple;
    (i->*F)(std::get<S>(tuple) ...);
}
}
#endif //USE_GCC47_WORKAROUND

class CSAPEX_QT_EXPORT QObserver : public QObject, public Observer
{
    Q_OBJECT

public:
    QObserver();

    ~QObserver();

    template <typename Lambda, typename Result, typename... Args>
    void observeQueued(slim_signal::Signal<Result(Args...)>& signal,
                       Lambda callback)
    {
        auto lambda = [=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);

#if USE_GCC47_WORKAROUND
                std::tuple<Args...> args_tuple(args...);
#endif //USE_GCC47_WORKAROUND

                observation_queue_.emplace_back([=](){
#if USE_GCC47_WORKAROUND
                    gcc47::call(typename gcc47::generateSequence<sizeof...(Args)>::type(), callback, args_tuple);
#else  //USE_GCC47_WORKAROUND
                    callback(args...);
#endif //USE_GCC47_WORKAROUND
                });
            }
            Q_EMIT handleObservationsRequest();
        };

        manageConnection(signal.connect(lambda));
    }

    template <typename Lambda, typename Result, typename... Args>
    void observeQueued(std::shared_ptr<slim_signal::Signal<Result(Args...)>>& signal,
                       Lambda callback)
    {
        auto fn = [=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);

#if USE_GCC47_WORKAROUND
                std::tuple<Args...> args_tuple(args...);
#endif //USE_GCC47_WORKAROUND

                observation_queue_.emplace_back([=](){
#if USE_GCC47_WORKAROUND
                    gcc47::call(typename gcc47::generateSequence<sizeof...(Args)>::type(), callback, args_tuple);
#else  //USE_GCC47_WORKAROUND
                    callback(args...);
#endif //USE_GCC47_WORKAROUND
                });
            }
            Q_EMIT handleObservationsRequest();
        };

        manageConnection(signal->connect(fn));
    }



    // MEMBER FUNCTIONS
    template <typename Receiver, typename Result, typename... Args>
    void observeQueued(slim_signal::Signal<Result(Args...)>& signal,
                       Receiver* instance,
                       Result (Receiver::*function)(Args...))
    {
        auto fn = [=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);

#if USE_GCC47_WORKAROUND
                std::tuple<Args...> args_tuple(args...);
#endif //USE_GCC47_WORKAROUND

                observation_queue_.emplace_back([=](){
#if USE_GCC47_WORKAROUND
                    gcc47::call(typename gcc47::generateSequence<sizeof...(Args)+1>::type(), instance, function, args_tuple);
#else  //USE_GCC47_WORKAROUND
                    (instance->*function)(args...);
#endif //USE_GCC47_WORKAROUND
                });
            }
            Q_EMIT handleObservationsRequest();
        };

        manageConnection(signal.connect(fn));
    }

    template <typename Receiver, typename Result, typename... Args>
    void observeQueued(std::shared_ptr<slim_signal::Signal<Result(Args...)>>& signal,
                       Receiver* instance,
                       Result (Receiver::*function)(Args...))
    {
        auto fn = [=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);

#if USE_GCC47_WORKAROUND
                std::tuple<Args...> args_tuple(args...);
#endif //USE_GCC47_WORKAROUND

                observation_queue_.emplace_back([=](){
#if USE_GCC47_WORKAROUND
                    gcc47::call(typename gcc47::generateSequence<sizeof...(Args)>::type(), instance, function, args_tuple);
#else  //USE_GCC47_WORKAROUND
                    (instance->*function)(args...);
#endif //USE_GCC47_WORKAROUND
                });
            }
            Q_EMIT handleObservationsRequest();
        };

        manageConnection(signal->connect(fn));
    }



Q_SIGNALS:
    void handleObservationsRequest();

private Q_SLOTS:
    void handleObservations()
    {
        std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);
        for(const auto& fn : observation_queue_) {
            fn();
        }
    }

private:
    std::recursive_mutex observation_queue_mutex_;
    std::vector<std::function<void()>> observation_queue_;
};

}


#endif // QOBSERVER_H
