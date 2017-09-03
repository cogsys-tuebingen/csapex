#ifndef CONNECTOR_REMOTE_H
#define CONNECTOR_REMOTE_H

/// PROJECT
#include <csapex/model/connector.h>
#include <csapex/io/session.h>

namespace csapex
{

class ConnectorRemote : public Connector, public Observer
{
public:
    ConnectorRemote(UUID uuid, ConnectableOwnerPtr owner, SessionPtr session,
                    ConnectorPtr tmp_connector);

    virtual bool canConnectTo(Connector* other_side, bool move) const override;
    virtual bool targetsCanBeMovedTo(Connector* other_side) const override;
    virtual bool isConnectionPossible(Connector* other_side) override;
    virtual void validateConnections() override;
    virtual void connectionMovePreview(ConnectorPtr other_side) override;

    virtual int getCount() const override;

    virtual bool canOutput() const override;
    virtual bool canInput() const override;
    virtual bool isOutput() const override;
    virtual bool isInput() const override;
    virtual bool isOptional() const override;

    virtual bool isSynchronous() const override;

    virtual bool isVirtual() const override;
    virtual void setVirtual(bool _virtual) override;

    virtual bool isParameter() const override;

    virtual bool isGraphPort() const override;
    virtual void setGraphPort(bool graph) override;

    virtual bool isEssential() const override;
    virtual void setEssential(bool essential) override;

    virtual void addConnection(ConnectionPtr connection) override;
    virtual void removeConnection(Connector* other_side) override;
    virtual void fadeConnection(ConnectionPtr connection) override;

    virtual void setLabel(const std::string& label) override;
    virtual std::string getLabel() const override;

    virtual void setType(TokenData::ConstPtr type) override;
    virtual TokenData::ConstPtr getType() const override;

    virtual ConnectorType getConnectorType() const override;

    virtual ConnectorDescription getDescription() const override;

    virtual bool isEnabled() const override;
    virtual void setEnabled(bool enabled) override;

    virtual int sequenceNumber() const override;
    virtual void setSequenceNumber(int seq_no_) override;

    virtual int countConnections() override;
    virtual std::vector<ConnectionPtr> getConnections() const override;

    virtual bool hasActiveConnection() const override;

    virtual bool isConnected() const override;

    virtual void disable() override;
    virtual void enable() override;

    virtual void reset() override;
    virtual void stop() override;

    virtual void notifyMessageProcessed() override;

    virtual std::string makeStatusString() const override;

private:
    SessionPtr session_;
    ConnectorPtr tmp_connector_;
};

}

#endif // CONNECTOR_REMOTE_H
