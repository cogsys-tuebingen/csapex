/// HEADER
#include <csapex/view/widgets/port_panel.h>

/// PROJECT
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node_facade.h>
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

    default:
        throw std::runtime_error("tried to create a connector of type 'NONE'");
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

    graph_facade_->forwardingAdded.connect(delegate::Delegate<void(ConnectorPtr)>(this, &PortPanel::connectorAdded));
    graph_facade_->forwardingRemoved.connect(delegate::Delegate<void(ConnectorPtr)>(this, &PortPanel::connectorRemoved));
}

void PortPanel::updateLayouts()
{
    mainlayout->activate();
    layout->activate();

    QApplication::processEvents();

    adjustSize();
}

void PortPanel::addPortForConnector(ConnectorPtr c)
{
    if(!c) {
        return;
    }
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


void PortPanel::removePortForConnector(ConnectorPtr c)
{
    if(c->getConnectorType() != type_) {
        return;
    }

    Port* port = parent_->getPort(c->getUUID());
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
    for(const ConnectorDescription& connector : graph_facade_->getNodeFacade()->getInternalOutputs()) {
        addPortForConnector(graph_facade_->findConnectorNoThrow(connector.id));
    }
}

void PortPanel::setupInput()
{
    for(const ConnectorDescription& connector : graph_facade_->getNodeFacade()->getInternalInputs()) {
        addPortForConnector(graph_facade_->findConnectorNoThrow(connector.id));
    }
}

void PortPanel::setupSlot()
{
    for(const ConnectorDescription& connector : graph_facade_->getNodeFacade()->getInternalSlots()) {
        addPortForConnector(graph_facade_->findConnectorNoThrow(connector.id));
    }
}

void PortPanel::setupEvent()
{
    for(const ConnectorDescription& connector : graph_facade_->getNodeFacade()->getInternalEvents()) {
        addPortForConnector(graph_facade_->findConnectorNoThrow(connector.id));
    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_port_panel.cpp"
