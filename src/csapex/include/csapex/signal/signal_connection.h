#ifndef SIGNAL_CONNECTION_H
#define SIGNAL_CONNECTION_H

/// PROJECT
#include <csapex/model/connection.h>

namespace csapex
{

class SignalConnection : public Connection
{
public:
    static ConnectionPtr connect(Event* from, Slot* to);
    static ConnectionPtr connect(Event* from, Slot* to, int id);

private:
    SignalConnection(Event* from, Slot* to);
    SignalConnection(Event* from, Slot* to, int id);
};

}

#endif // SIGNAL_CONNECTION_H

