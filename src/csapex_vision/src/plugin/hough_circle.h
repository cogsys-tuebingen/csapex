#ifndef HOUGH_CIRCLE_H
#define HOUGH_CIRCLE_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class HoughCircle : public Node
{
public:
    HoughCircle();

    virtual void setup();
    virtual void allConnectorsArrived();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

}

#endif // HOUGH_CIRCLE_H
