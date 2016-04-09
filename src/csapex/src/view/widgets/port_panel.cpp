/// HEADER
#include <csapex/view/widgets/port_panel.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/view/widgets/port.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/widgets/meta_port.h>

/// SYSTEM
#include <QBoxLayout>
#include <QMimeData>
#include <QApplication>
#include <iostream>

using namespace csapex;


PortPanel::PortPanel(ConnectorType type, DesignerScene* parent)
    : type_(type), parent_(parent)
{
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setMinimumSize(10, 10);

    QBoxLayout* mainlayout = nullptr;

    setFocusPolicy(Qt::NoFocus);

    switch(type) {
    case ConnectorType::INPUT:
    case ConnectorType::OUTPUT:
        mainlayout =  new QVBoxLayout;;
        layout = new QVBoxLayout;

        break;


    case ConnectorType::SLOT_T:
    case ConnectorType::TRIGGER:
        mainlayout =  new QHBoxLayout;;
        layout = new QHBoxLayout;

        break;
    }

    mainlayout->addLayout(layout);

    MetaPort* meta_port = new MetaPort(port_type::opposite(type_));
    QObject::connect(meta_port, &MetaPort::createPortRequest, this, &PortPanel::createPortRequest);
    QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &PortPanel::createPortAndConnectRequest);
    QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &PortPanel::createPortAndMoveRequest);
    mainlayout->addWidget(meta_port);

    setLayout(mainlayout);

    QObject::connect(this, &PortPanel::connectorAdded, this, &PortPanel::addPortForConnector);
    QObject::connect(this, &PortPanel::connectorRemoved, this, &PortPanel::removePortForConnector);
}

void PortPanel::setup(GraphFacadePtr graph_facade)
{
    graph_facade_ = graph_facade;
    graph_ = graph_facade_->getGraph();

    switch(type_) {
    case ConnectorType::OUTPUT:
        setupOutput();
        break;
    case ConnectorType::INPUT:
        setupInput();
        break;
    case ConnectorType::SLOT_T:
        setupSlot();
        break;
    case ConnectorType::TRIGGER:
        setupTrigger();
        break;

    default:
        throw std::logic_error("unsupported type");
    }

    graph_->forwardingAdded.connect(delegate::Delegate<void(ConnectablePtr)>(this, &PortPanel::connectorAdded));
    graph_->forwardingRemoved.connect(delegate::Delegate<void(ConnectablePtr)>(this, &PortPanel::connectorRemoved));
}

void PortPanel::addPortForConnector(ConnectablePtr c)
{
    if(c->getConnectorType() != type_) {
        return;
    }

    Port* port = new Port(c);

    parent_->addPort(port);

    layout->addWidget(port);

    layout->activate();

    QApplication::processEvents();

    adjustSize();
    portAdded(port);
}


void PortPanel::removePortForConnector(ConnectablePtr c)
{
    if(c->getConnectorType() != type_) {
        return;
    }

    Port* port = parent_->getPort(c.get());
    port->setVisible(false);

    apex_assert_hard(port);

    parent_->removePort(port);
    layout->removeWidget(port);

    layout->activate();

    adjustSize();
    portRemoved(port);
}

void PortPanel::setupOutput()
{
    for(const UUID& uuid : graph_->getRelayOutputs()) {
        addPortForConnector(graph_->getForwardedOutputInternal(uuid));
    }
}

void PortPanel::setupInput()
{
    for(const UUID& uuid : graph_->getRelayInputs()) {
        addPortForConnector(graph_->getForwardedInputInternal(uuid));
    }
}

void PortPanel::setupSlot()
{
    for(const UUID& uuid : graph_->getRelaySlots()) {
        addPortForConnector(graph_->getForwardedSlotInternal(uuid));
    }
}

void PortPanel::setupTrigger()
{
    for(const UUID& uuid : graph_->getRelayTriggers()) {
        addPortForConnector(graph_->getForwardedTriggerInternal(uuid));
    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_port_panel.cpp"
