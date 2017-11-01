#ifndef QOBSERVER_H
#define QOBSERVER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <mutex>
#include <QObject>

namespace csapex
{

class CSAPEX_QT_EXPORT QObserver : public QObject, public Observer
{
    Q_OBJECT

public:
    QObserver();


    template <typename Lambda, typename Result, typename... Args>
    void observeQueued(slim_signal::Signal<Result(Args...)>& signal,
                       Lambda callback)
    {
        manageConnection(signal.connect([=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);
                observation_queue_.emplace_back([=](){
                    callback(args...);
                });
            }
            Q_EMIT handleObservationsRequest();
        }));
    }

    template <typename Lambda, typename Result, typename... Args>
    void observeQueued(std::shared_ptr<slim_signal::Signal<Result(Args...)>>& signal,
                       Lambda callback)
    {
        manageConnection(signal->connect([=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);
                observation_queue_.emplace_back([=](){
                    callback(args...);
                });
            }
            Q_EMIT handleObservationsRequest();
        }));
    }



    // MEMBER FUNCTIONS
    template <typename Receiver, typename Result, typename... Args>
    void observeQueued(slim_signal::Signal<Result(Args...)>& signal,
                       Receiver* instance,
                       Result (Receiver::*function)(Args...))
    {
        manageConnection(signal.connect([=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);
                observation_queue_.emplace_back([=](){
                    (instance->*function)(args...);
                });
            }
            Q_EMIT handleObservationsRequest();
        }));
    }

    template <typename Receiver, typename Result, typename... Args>
    void observeQueued(std::shared_ptr<slim_signal::Signal<Result(Args...)>>& signal,
                       Receiver* instance,
                       Result (Receiver::*function)(Args...))
    {
        manageConnection(signal->connect([=](Args... args) {
            {
                std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);
                observation_queue_.emplace_back([=](){
                    (instance->*function)(args...);
                });
            }
            Q_EMIT handleObservationsRequest();
        }));
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
