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
    BundledConnection(Output* from, Input* to);
    BundledConnection(Output* from, Input* to, int id);
};

}

#endif // BUNDLED_CONNECTION_H

