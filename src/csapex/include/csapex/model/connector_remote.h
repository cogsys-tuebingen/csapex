#ifndef CONNECTOR_REMOTE_H
#define CONNECTOR_REMOTE_H

/// PROJECT
#include <csapex/model/connector.h>
#include <csapex/io/session.h>
#include <csapex/io/remote.h>


namespace csapex
{

class ConnectorRemote : public Connector, public Observer, public Remote
{
public:
    ConnectorRemote(UUID uuid, ConnectableOwnerPtr owner, SessionPtr session,
                    ConnectorPtr tmp_connector);

    virtual bool isCompatibleWith(Connector* other_side) const override;
    virtual bool targetsCanBeMovedTo(Connector* other_side) const override;
    virtual bool isConnectionPossible(Connector* other_side) override;
    virtual void connectionMovePreview(ConnectorPtr other_side) override;

    virtual int getCount() const override;

    virtual bool isOutput() const override;
    virtual bool isInput() const override;
    virtual bool isOptional() const override;
    virtual bool isSynchronous() const override;
    virtual bool isVirtual() const override;
    virtual bool isParameter() const override;
    virtual bool isGraphPort() const override;
    virtual bool isEssential() const override;

    virtual std::string getLabel() const override;
    virtual TokenData::ConstPtr getType() const override;
    virtual ConnectorType getConnectorType() const override;
    virtual ConnectorDescription getDescription() const override;

    virtual bool isEnabled() const override;
    virtual int sequenceNumber() const override;
    virtual int countConnections() const override;
    virtual bool hasActiveConnection() const override;
    virtual bool isConnected() const override;

    virtual std::string makeStatusString() const override;

private:
    ConnectorPtr tmp_connector_;
};

}

#endif // CONNECTOR_REMOTE_H
