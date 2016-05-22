#ifndef PORT_PANEL_H
#define PORT_PANEL_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/connector_type.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/create_connector_request.h>

/// SYSTEM
#include <QFrame>

class QBoxLayout;

namespace csapex
{

class PortPanel : public QFrame
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:

public:
    PortPanel(ConnectorType type, DesignerScene *parent);

    void setup(GraphFacadePtr graph_facade);

    void enableMetaPort(const AUUID& target);

    QString cssClass() {
        return QString("PortPanel");
    }

Q_SIGNALS:
    void createPortRequest(CreateConnectorRequest request);
    void createPortAndConnectRequest(CreateConnectorRequest request, Connectable*);
    void createPortAndMoveRequest(CreateConnectorRequest request, Connectable*);

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
    void setupEvent();

    void updateLayouts();

private:
    GraphFacadePtr graph_facade_;
    Graph* graph_;
    ConnectorType type_;
    DesignerScene *parent_;

    QBoxLayout* mainlayout;
    QLayout* layout;
};

}
#endif // PORT_PANEL_H
