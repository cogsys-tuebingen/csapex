#ifndef EXPORTROS_H
#define EXPORTROS_H

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QLineEdit>
#include <ros/publisher.h>

namespace csapex {

class ExportRos : public BoxedObject
{
    Q_OBJECT

public:
    ExportRos();

    virtual void fill(QBoxLayout* layout);
    virtual void process();

private Q_SLOTS:
    void updateTopic();

private:
    ConnectorIn* connector_;

    struct State : public Memento {
        std::string topic;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    bool has_pub;
    bool create_pub;

    State state;
    ros::Publisher pub;

    QLineEdit* topic_;
};

}
#endif // EXPORTROS_H
