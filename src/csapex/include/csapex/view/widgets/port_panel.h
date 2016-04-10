#ifndef PORT_PANEL_H
#define PORT_PANEL_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/connector_type.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QFrame>

namespace csapex
{

class PortPanel : public QFrame
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:

public:
    PortPanel(ConnectorType type, const AUUID &target, DesignerScene *parent);

    void setup(GraphFacadePtr graph_facade);

    QString cssClass() {
        return QString("PortPanel");
    }

Q_SIGNALS:
    void createPortRequest(const AUUID& target, ConnectorType type, ConnectionTypeConstPtr, std::string, bool);
    void createPortAndConnectRequest(const AUUID& target, Connectable*, ConnectionTypeConstPtr, std::string, bool);
    void createPortAndMoveRequest(const AUUID& target, Connectable*, ConnectionTypeConstPtr, std::string, bool);

    void portAdded(Port*);
    void portRemoved(Port*);

    void connectorAdded(ConnectablePtr c);
    void connectorRemoved(ConnectablePtr c);

private Q_SLOTS:
    void addPortForConnector(ConnectablePtr c);
    void removePortForConnector(ConnectablePtr c);

private:
    void setupOutput();
    void setupInput();
    void setupSlot();
    void setupTrigger();

private:
    GraphFacadePtr graph_facade_;
    Graph* graph_;
    ConnectorType type_;
    DesignerScene *parent_;

    QLayout* layout;
};

}
#endif // PORT_PANEL_H
