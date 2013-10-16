#ifndef LISTENER_H
#define LISTENER_H

/// PROJECT
#include <csapex_core_plugins/ros_handler.h>

/// SYSTEM
#include <tf/transform_listener.h>

namespace csapex
{

struct Listener {
    boost::shared_ptr<tf::TransformListener> tfl;

    static Listener* instance() {
        Listener* l = raw_instance();
        if(l->ok()) {
            return l;
        } else {
            return NULL;
        }
    }

    static void start();

    void reset() {
        //tfl.clear();
        tfl.reset(new tf::TransformListener);
    }

private:

    static Listener* raw_instance() {
        static Listener l;
        assert(&l);
        return &l;
    }

    Listener();

    bool ok();
    void cb(const tf::tfMessage::ConstPtr& msg);

    int retries;
    std::string reference_frame;
    bool init;
    ros::Time last_;
    ros::Subscriber tf_sub;
};

}

#endif // LISTENER_H
