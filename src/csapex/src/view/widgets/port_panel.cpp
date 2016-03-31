/// HEADER
#include <csapex/view/widgets/port_panel.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
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


PortPanel::PortPanel(Type type, DesignerScene* parent)
    : type_(type), parent_(parent)
{
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setMinimumSize(10, 10);

    QVBoxLayout* mainlayout = new QVBoxLayout;

    setFocusPolicy(Qt::NoFocus);

    layout = new QVBoxLayout;
    mainlayout->addLayout(layout);

    MetaPort* meta_port = new MetaPort(type_ != Type::OUTPUT_RELAY);
    QObject::connect(meta_port, &MetaPort::createPortRequest, this, &PortPanel::createPortRequest);
    QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &PortPanel::createPortAndConnectRequest);
    QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &PortPanel::createPortAndMoveRequest);
    mainlayout->addWidget(meta_port);

    setLayout(mainlayout);
}

void PortPanel::setup(GraphFacadePtr graph_facade)
{
    graph_facade_ = graph_facade;
    graph_ = graph_facade_->getGraph();

    switch(type_) {
    case Type::OUTPUT_RELAY:
        setupOutput();
        break;
    case Type::INPUT_RELAY:
        setupInput();
        break;

    default:
        throw std::logic_error("unsupported type");
    }

    graph_->forwardingAdded.connect(delegate::Delegate<void(ConnectablePtr)>(this, &PortPanel::connectorAdded));
}

void PortPanel::connectorAdded(ConnectablePtr c)
{
    switch(type_) {
    case Type::OUTPUT_RELAY:
        if(std::dynamic_pointer_cast<Output>(c)) {
            add(c);
        }
        break;
    case Type::INPUT_RELAY:
        if(std::dynamic_pointer_cast<Input>(c)) {
            add(c);
        }
        break;

    default:
        break;
    }
}

void PortPanel::setupOutput()
{
    for(const UUID& uuid : graph_->getRelayOutputs()) {
        add(graph_->getForwardedOutputInternal(uuid));
    }
}

void PortPanel::setupInput()
{
    for(const UUID& uuid : graph_->getRelayInputs()) {
        add(graph_->getForwardedInputInternal(uuid));
    }
}

void PortPanel::add(ConnectablePtr c)
{
    Port* port = new Port(c);

    ports_.push_back(port);
    parent_->addPort(port);

    layout->addWidget(port);
    QApplication::processEvents();

    adjustSize();

    QObject::connect(port, &Port::destroyed, [this, port](QObject* o) {
        portRemoved(port);
    });
    portAdded(port);
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_port_panel.cpp"
