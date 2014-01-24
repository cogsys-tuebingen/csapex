#ifndef DisplayFeatures_H
#define DisplayFeatures_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{

class DisplayKeypoints : public csapex::Node
{
public:
    DisplayKeypoints();

public:
    virtual void setup();

    virtual void allConnectorsArrived();

private:
    ConnectorIn* in_img;
    ConnectorIn* in_key;

    ConnectorOut* out_img;
};

}

#endif // DisplayFeatures_H
