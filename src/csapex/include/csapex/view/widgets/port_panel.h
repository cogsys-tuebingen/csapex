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
        OUTPUT, INPUT, TRIGGER, SLOT_
    };

public:
    PortPanel(GraphFacadePtr graph_facade, Type type, DesignerScene *parent);

    QString cssClass() {
        return QString("PortPanel");
    }

//    void dragEnterEvent(QDragEnterEvent* e);
//    void dragMoveEvent(QDragMoveEvent* e);
//    void dropEvent(QDropEvent* e);

Q_SIGNALS:
    void createPortRequest(Connectable*, ConnectionTypeConstPtr, std::string, bool);

public Q_SLOTS:
    void connectorAdded(ConnectablePtr c);

private:
    void setupOutput();
    void setupInput();

    void setup();

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
