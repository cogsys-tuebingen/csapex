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
#include <QDragEnterEvent>
#include <QDrag>
#include <QApplication>
#include <iostream>

using namespace csapex;


PortPanel::PortPanel(GraphFacadePtr graph_facade, Type type, DesignerScene* parent)
    : graph_facade_(graph_facade), type_(type), parent_(parent)
{
    graph_ = graph_facade_->getGraph();

    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setMinimumSize(10, 10);

    QVBoxLayout* mainlayout = new QVBoxLayout;


    layout = new QVBoxLayout;
    mainlayout->addLayout(layout);

    MetaPort* meta_port = new MetaPort;
    QObject::connect(meta_port, &MetaPort::createPortRequest, this, &PortPanel::createPortRequest);
    mainlayout->addWidget(meta_port);

    setLayout(mainlayout);

    setAcceptDrops(true);

    setup();

    graph_->forwardingAdded.connect(delegate::Delegate<void(ConnectablePtr)>(this, &PortPanel::connectorAdded));
}

void PortPanel::connectorAdded(ConnectablePtr c)
{
    switch(type_) {
    case Type::OUTPUT:
        if(std::dynamic_pointer_cast<Output>(c)) {
            add(c);
        }
        break;
    case Type::INPUT:
        if(std::dynamic_pointer_cast<Input>(c)) {
            add(c);
        }
        break;

    default:
        break;
    }
}

void PortPanel::setup()
{
    switch(type_) {
    case Type::OUTPUT:
        setupOutput();
        break;
    case Type::INPUT:
        setupInput();
        break;

    default:
        throw std::logic_error("unsupported type");
    }

    if(ports_.empty()) {
        hide();
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
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_port_panel.cpp"
