#ifndef EXPORTROS_H
#define EXPORTROS_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <ros/publisher.h>

namespace csapex {

class ExportRos : public Node
{
    Q_OBJECT

public:
    ExportRos();

    void setup();
    virtual void process();

    virtual QIcon getIcon() const;

protected:
    void updateTopic();

private:
    ConnectorIn* connector_;

    bool create_pub;

    ros::Publisher pub;

    std::string topic_;
};

}
#endif // EXPORTROS_H
