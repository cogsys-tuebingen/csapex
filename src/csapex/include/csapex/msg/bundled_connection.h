#ifndef BUNDLED_CONNECTION_H
#define BUNDLED_CONNECTION_H

/// PROJECT
#include <csapex/model/connection.h>

namespace csapex
{

class BundledConnection : public Connection
{
public:
    static ConnectionPtr connect(Output* from, Input* to, OutputTransition* ot, InputTransition* it);
    static ConnectionPtr connect(Output* from, Input* to, OutputTransition* ot, InputTransition* it, int id);

private:
    BundledConnection(Output* from, Input* to, OutputTransition* ot, InputTransition* it);
    BundledConnection(Output* from, Input* to, OutputTransition* ot, InputTransition* it, int id);

    virtual void setMessage(const ConnectionTypeConstPtr &msg) override;

    virtual void establishSource() override;
    virtual void establishSink() override;

private:
    OutputTransition* ot_;
    InputTransition* it_;
};

}

#endif // BUNDLED_CONNECTION_H

