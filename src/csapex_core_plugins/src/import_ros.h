#ifndef IMPORT_ROS_H
#define IMPORT_ROS_H

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <ros/ros.h>
#include <QComboBox>

namespace csapex {

class ImportRos : public BoxedObject
{
    Q_OBJECT

public:
    ImportRos();

    virtual void fill(QBoxLayout* layout);
    virtual void updateDynamicGui(QBoxLayout *layout);
    virtual void messageArrived(ConnectorIn* source);
    virtual void tick();

Q_SIGNALS:
    void newTopic(const ros::master::TopicInfo& topic);

public Q_SLOTS:
    void refresh();
    void changeTopic(const QString &topic);

public:
    struct State : public Memento {
        typedef boost::shared_ptr<State> Ptr;

        std::string topic_;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

private:
    State state;

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private Q_SLOTS:
    void setTopic(const ros::master::TopicInfo& topic);

private:
    ConnectorOut* connector_;

    QVBoxLayout* dynamic_layout;
    QComboBox* topic_list;

    ros::Subscriber current_subscriber;
};

}


#endif // IMPORT_ROS_H
