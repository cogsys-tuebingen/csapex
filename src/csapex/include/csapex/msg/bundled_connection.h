#ifndef BUNDLED_CONNECTION_H
#define BUNDLED_CONNECTION_H

/// PROJECT
#include <csapex/msg/direct_connection.h>

namespace csapex
{

class CSAPEX_EXPORT BundledConnection : public DirectConnection
{
public:
    static ConnectionPtr connect(Output* from, Input* to);
    static ConnectionPtr connect(Output* from, Input* to, int id);

private:
    BundledConnection(Output* from, Input* to);
    BundledConnection(Output* from, Input* to, int id);
};

}

#endif // BUNDLED_CONNECTION_H

