#ifndef THROTTLED_NODE_H
#define THROTTLED_NODE_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class CSAPEX_EXPORT ThrottledNode : public Node
{
public:
    ThrottledNode();

protected:
    void setupParameter(csapex::Parameterizable& params, const std::string& name);
};

}

#endif // THROTTLED_NODE_H
