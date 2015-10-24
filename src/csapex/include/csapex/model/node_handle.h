#ifndef NODE_HANDLE_H
#define NODE_HANDLE_H

namespace csapex
{

class NodeHandle
{
public:
    virtual ~NodeHandle();

    virtual bool canProcess() const = 0;
    virtual bool isEnabled() const = 0;

    virtual void triggerCheckTransitions() = 0;
    virtual void startProcessingMessages() = 0;
};

}

#endif // NODE_HANDLE_H

