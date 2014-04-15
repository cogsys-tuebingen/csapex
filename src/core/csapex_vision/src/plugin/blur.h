#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class BoxBlur : public Node
{
public:
    BoxBlur();

    virtual void process();
    virtual void setup();

private:
    void update();

    ConnectorIn  *input_;
    ConnectorOut *output_;

    int           kernel_;
};

} /// NAMESPACE

#endif // FilterBlur_H
