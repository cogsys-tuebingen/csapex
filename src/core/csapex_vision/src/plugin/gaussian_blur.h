#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class GaussianBlur : public Node
{
public:
    GaussianBlur();

    virtual void process();
    virtual void setup();

private:
    void update();

    ConnectorIn  *input_;
    ConnectorOut *output_;

    int           kernel_;
    double        sigma_x_;
    double        sigma_y_;
};

} /// NAMESPACE

#endif // FilterBlur_H
