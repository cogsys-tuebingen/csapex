/// HEADER
#include <csapex/view/widgets/port_panel.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/model/subgraph_node.h>
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
    : type_(type), parent_(parent), mainlayout(nullptr), layout(nullptr)
{
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setMinimumSize(10, 10);

    setFocusPolicy(Qt::NoFocus);

    switch(type) {
    case ConnectorType::INPUT:
    case ConnectorType::OUTPUT:
        mainlayout =  new QVBoxLayout;;
        mainlayout->setSpacing(16);
        layout = new QVBoxLayout;
        layout->setSpacing(16);

        break;


    case ConnectorType::SLOT_T:
    case ConnectorType::EVENT:
        mainlayout =  new QHBoxLayout;;
        mainlayout->setSpacing(32);
        layout = new QHBoxLayout;
        layout->setSpacing(32);

        break;
    }

    mainlayout->setMargin(2);

    mainlayout->addLayout(layout);

    setVisible(false);

    setLayout(mainlayout);

    QObject::connect(this, &PortPanel::connectorAdded, this, &PortPanel::addPortForConnector);
    QObject::connect(this, &PortPanel::connectorRemoved, this, &PortPanel::removePortForConnector);
}

void PortPanel::setVisible(bool visible)
{
    if(mainlayout->isEmpty()) {
        QFrame::setVisible(false);
    } else {
        QFrame::setVisible(visible);
    }
}

void PortPanel::enableMetaPort(const AUUID& target)
{
    MetaPort* meta_port = new MetaPort(port_type::opposite(type_), target);
    QObject::connect(meta_port, &MetaPort::createPortRequest, this, &PortPanel::createPortRequest);
    QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &PortPanel::createPortAndConnectRequest);
    QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &PortPanel::createPortAndMoveRequest);

    mainlayout->addWidget(meta_port);

    updateLayouts();

    setVisible(true);
}

void PortPanel::setup(GraphFacadePtr graph_facade)
{
    graph_facade_ = graph_facade;
    graph_ = graph_facade_->getSubgraphNode();

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
    case ConnectorType::EVENT:
        setupEvent();
        break;

    default:
        throw std::logic_error("unsupported type");
    }

    graph_->forwardingAdded.connect(delegate::Delegate<void(ConnectablePtr)>(this, &PortPanel::connectorAdded));
    graph_->forwardingRemoved.connect(delegate::Delegate<void(ConnectablePtr)>(this, &PortPanel::connectorRemoved));
}

void PortPanel::updateLayouts()
{
    mainlayout->activate();
    layout->activate();

    QApplication::processEvents();

    adjustSize();
}

void PortPanel::addPortForConnector(ConnectablePtr c)
{
    if(c->getConnectorType() != type_) {
        return;
    }

    Port* port = new Port(c);

    parent_->addPort(port);

    layout->addWidget(port);

    updateLayouts();

    portAdded(port);

    setVisible(true);
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
    for(const UUID& uuid : graph_->getInternalOutputs()) {
        addPortForConnector(graph_->getForwardedOutputInternal(uuid));
    }
}

void PortPanel::setupInput()
{
    for(const UUID& uuid : graph_->getInternalInputs()) {
        addPortForConnector(graph_->getForwardedInputInternal(uuid));
    }
}

void PortPanel::setupSlot()
{
    for(const UUID& uuid : graph_->getInternalSlots()) {
        addPortForConnector(graph_->getForwardedSlotInternal(uuid));
    }
}

void PortPanel::setupEvent()
{
    for(const UUID& uuid : graph_->getInternalEvents()) {
        addPortForConnector(graph_->getForwardedEventInternal(uuid));
    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_port_panel.cpp"
