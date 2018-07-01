#ifndef THROTTLED_NODE_H
#define THROTTLED_NODE_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_CORE_EXPORT ThrottledNode : public Node
{
public:
    ThrottledNode();

protected:
    void setupParameter(csapex::Parameterizable& params, const std::string& name);
};

}  // namespace csapex

#endif  // THROTTLED_NODE_H
