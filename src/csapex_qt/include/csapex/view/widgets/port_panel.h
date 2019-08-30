#ifndef PORT_PANEL_H
#define PORT_PANEL_H

/// COMPONENT
#include <csapex_qt/export.h>

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/connector_type.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_description.h>

/// SYSTEM
#include <QFrame>

class QBoxLayout;

namespace csapex
{
class CSAPEX_QT_EXPORT PortPanel : public QFrame
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
public:
    PortPanel(ConnectorType type, DesignerScene* parent);

    void setup(GraphFacadePtr graph_facade);

    void enableMetaPort(const AUUID& target);

    QString cssClass()
    {
        return QString("PortPanel");
    }

Q_SIGNALS:
    void createPortRequest(ConnectorDescription request);
    void createPortAndConnectRequest(ConnectorDescription request, ConnectorPtr);
    void createPortAndMoveRequest(ConnectorDescription request, ConnectorPtr);

    void portAdded(Port*);
    void portRemoved(Port*);

    void connectorAdded(ConnectorPtr c);
    void connectorRemoved(ConnectorPtr c);

public Q_SLOTS:
    virtual void setVisible(bool visible);

private Q_SLOTS:
    void addPortForConnector(ConnectorPtr c);
    void removePortForConnector(ConnectorPtr c);

private:
    void setupOutput();
    void setupInput();
    void setupSlot();
    void setupEvent();

    void updateLayouts();

private:
    GraphFacadePtr graph_facade_;
    ConnectorType type_;
    DesignerScene* parent_;

    QBoxLayout* mainlayout;
    QLayout* layout;
};

}  // namespace csapex
#endif  // PORT_PANEL_H
