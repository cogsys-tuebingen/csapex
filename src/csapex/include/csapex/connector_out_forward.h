#ifndef CONNECTOR_OUT_FORWARD_H
#define CONNECTOR_OUT_FORWARD_H

/// COMPONENT
#include "connector_out.h"

namespace csapex
{

class ConnectorOutForward : public ConnectorOut
{
    Q_OBJECT

public:
    ConnectorOutForward(Box* parent, const std::string& uuid);
    ConnectorOutForward(Box* parent, int sub_id);

    virtual bool isForwarding() const;

public Q_SLOTS:
    void relayMessage(ConnectorOut* source);

};

}

#endif // CONNECTOR_OUT_FORWARD_H
