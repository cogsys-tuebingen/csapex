#ifndef PORT_PANEL_H
#define PORT_PANEL_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QFrame>

namespace csapex
{

class PortPanel : public QFrame
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    enum class Type {
        OUTPUT_RELAY, INPUT_RELAY
    };

public:
    PortPanel(Type type, DesignerScene *parent);

    void setup(GraphFacadePtr graph_facade);

    QString cssClass() {
        return QString("PortPanel");
    }


Q_SIGNALS:
    void createPortRequest(bool output, ConnectionTypeConstPtr, std::string, bool);
    void createPortAndConnectRequest(Connectable*, ConnectionTypeConstPtr, std::string, bool);
    void createPortAndMoveRequest(Connectable*, ConnectionTypeConstPtr, std::string, bool);

    void portAdded(Port*);
    void portRemoved(Port*);

public Q_SLOTS:
    void connectorAdded(ConnectablePtr c);

private:
    void setupOutput();
    void setupInput();

    void add(ConnectablePtr c);

private:
    GraphFacadePtr graph_facade_;
    Graph* graph_;
    Type type_;
    DesignerScene *parent_;

    QLayout* layout;

    std::vector<Port*> ports_;
};

}
#endif // PORT_PANEL_H
