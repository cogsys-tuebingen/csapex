#ifndef DYNAMIC_TRANSFORM_H
#define DYNAMIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_core_plugins/ros_handler.h>

/// SYSTEM
#include <QComboBox>
#include <QPushButton>
#include <tf/transform_listener.h>

namespace csapex {

class DynamicTransform : public csapex::BoxedObject
{
    Q_OBJECT

public:
    DynamicTransform();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);


public Q_SLOTS:
    virtual void allConnectorsArrived();
    virtual void tick();
    void update();
    void updateFrames();
    void resetTf();

private:
    void publishTransform(const ros::Time& time);

    struct Listener {
        boost::shared_ptr<tf::TransformListener> tfl;

        static Listener* instance() {
            if(!ROSHandler::instance().isConnected()) {
                return NULL;
            }
            static Listener l; return &l;
        }

        void reset() {
            //tfl.clear();
            tfl.reset(new tf::TransformListener);
        }

    private:
        Listener() {
            init = false;
            tf_sub = ROSHandler::instance().nh()->subscribe<tf::tfMessage>("/tf", 1, boost::bind(&Listener::cb, this, _1));
            tfl.reset(new tf::TransformListener);
            retries = 10;
        }

        void cb(const tf::tfMessage::ConstPtr& msg) {
            ros::Time now;
            if(init) {
                for(unsigned i = 0, n = msg->transforms.size(); i < n; ++i) {
                    if(msg->transforms[i].child_frame_id == reference_frame) {
                        now = msg->transforms[i].header.stamp;
                        if(last_ > now) {
                            std::cout << "warning: reset tf listener, negative time change (" << last_ << " vs. " << now << ")" << std::endl;
                            reset();
                        }

                        last_ = now;

                        break;
                    }
                }
            } else {
                // try to use /base_link as reference frame, if it exists
                reference_frame = "";
                std::string preferred = "/base_link";
                for(unsigned i = 0, n = msg->transforms.size(); i < n; ++i) {
                    if(msg->transforms[i].child_frame_id == preferred) {
                        reference_frame = preferred;
                        init = true;

                        last_ = msg->transforms[i].header.stamp;
                        return;
                    }
                }
                // if /base_link doesn't exist, use the first random one

                if(--retries <= 0) {
                    if(msg->transforms.empty()) {
                        std::cerr << "warning: no tf frames available!" << std::endl;
                        init = false;
                    } else {
                        reference_frame = msg->transforms[0].child_frame_id;
                        std::cerr << "warning: " << preferred << " frame not available! using " << reference_frame << std::endl;
                        init = true;

                        last_ = msg->transforms[0].header.stamp;
                    }
                }
            }
        }
        int retries;
        std::string reference_frame;
        bool init;
        ros::Time last_;
        ros::Subscriber tf_sub;
    };

private:
    ConnectorOut* output_;
    ConnectorOut* output_frame_;

    ConnectorIn* frame_in_from_;
    ConnectorIn* frame_in_to_;
    ConnectorIn* time_in_;

    QComboBox* from_box_;
    QComboBox* to_box_;

    QPushButton* refresh_;
    QPushButton* reset_tf_;

    struct State : public Memento {
        std::string from_;
        std::string to_;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // DYNAMIC_TRANSFORM_H
