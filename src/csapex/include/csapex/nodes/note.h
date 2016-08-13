#ifndef NOTE_H
#define NOTE_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class CSAPEX_EXPORT Note : public Node
{
public:
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    bool isIsolated() const override;
};

}

#endif // NOTE_H
