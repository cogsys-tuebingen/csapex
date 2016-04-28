#ifndef BUNDLED_CONNECTION_H
#define BUNDLED_CONNECTION_H

/// PROJECT
#include <csapex/msg/direct_connection.h>

namespace csapex
{

class BundledConnection : public DirectConnection
{
public:
    static ConnectionPtr connect(Output* from, Input* to, OutputTransition* ot, InputTransition* it);
    static ConnectionPtr connect(Output* from, Input* to, OutputTransition* ot, InputTransition* it, int id);

    static ConnectionPtr connect(Output* from, Input* to, OutputTransition* ot);
    static ConnectionPtr connect(Output* from, Input* to, InputTransition* it);

private:
    BundledConnection(Output* from, Input* to, OutputTransition* ot, InputTransition* it);
    BundledConnection(Output* from, Input* to, OutputTransition* ot, InputTransition* it, int id);

    virtual void setMessage(const TokenConstPtr &msg) override;

private:
    OutputTransition* ot_;
    InputTransition* it_;
};

}

#endif // BUNDLED_CONNECTION_H

