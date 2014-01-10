#ifndef LISTENER_H
#define LISTENER_H

/// PROJECT
#include <csapex_core_plugins/ros_handler.h>

/// SYSTEM
#include <tf/transform_listener.h>

namespace csapex
{

struct LockedListener;

struct Listener {
    friend class LockedListener;

public:
    boost::shared_ptr<tf::TransformListener> tfl;

    static LockedListener getLocked();

    static void start();

    void reset() {
        //tfl.clear();
        tfl.reset(new tf::TransformListener);
    }
    bool ok();

private:
    static Listener* raw_instance() {
        static Listener l;
        assert(&l);
        return &l;
    }

    Listener();

    void cb(const tf::tfMessage::ConstPtr& msg);

    int retries;
    std::string reference_frame;
    bool init;
    ros::Time last_;
    ros::Subscriber tf_sub;

    QMutex m;
};

struct LockedListener {
public:
    Listener* l;

    LockedListener(Listener* ll)
        : l(NULL)
    {
        if(ll) {
            ll->m.lock();
            l = ll;
        }
    }

    ~LockedListener()
    {
        if(l) {
            l->m.unlock();
        }
    }
};

}

#endif // LISTENER_H
