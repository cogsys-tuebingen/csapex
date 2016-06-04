/// HEADER
#include <csapex/view/widgets/rewiring_dialog.h>

/// COMPONENT
#include <csapex/view/designer/graph_view.h>
#include <csapex/core/csapex_core.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/view/widgets/box_dialog.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node_handle.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>
#include <csapex/msg/direct_connection.h>

/// SYSTEM
#include <QLabel>
#include <QKeyEvent>
#include <QListView>
#include <QVBoxLayout>
#include <QDialogButtonBox>

using namespace csapex;


RewiringDialog::RewiringDialog(GraphFacade *graph, NodeHandle* node, CsApexViewCore& view_core, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f),
      view_core_(view_core),

      graph_(graph),
      node_(node)
{
    root_uuid_provider_ = std::make_shared<UUIDProvider>();
}

RewiringDialog::~RewiringDialog()
{
//    graph_facade_old_.reset();
//    graph_facade_new_.reset();
//    graph_old.reset();
//    graph_new.reset();

//    executor.reset();
}

void RewiringDialog::makeUI(const QString& stylesheet)
{
    BoxDialog diag("Please enter the new node type.", view_core_.getCore().getNodeFactory(), view_core_.getNodeAdapterFactory());
    diag.setWindowTitle("Select new node type.");

    int r = diag.exec();

    if(!r) {
        return;
    }

    setStyleSheet(stylesheet);

    type_new_ = diag.getName();

    setWindowIcon(QIcon(":/pencil.png"));
    setWindowTitle(QString("Change Node to ") + QString::fromStdString(type_new_));

    setFocusPolicy(Qt::StrongFocus);
    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);


    NodeFactory& node_factory = view_core_.getCore().getNodeFactory();
    executor = std::make_shared<ThreadPool>(view_core_.getCore().getExceptionHandler(), false, false);


    graph_old_handle = node_factory.makeNode("csapex::Graph", UUIDProvider::makeUUID_without_parent("~"), root_uuid_provider_.get());
    apex_assert_hard(graph_old_handle);
    graph_old = std::dynamic_pointer_cast<Graph>(graph_old_handle->getNode().lock());
    apex_assert_hard(graph_old);
    graph_facade_old_ = std::make_shared<GraphFacade>(*executor, graph_old.get(), graph_old_handle.get());

    graph_new_handle = node_factory.makeNode("csapex::Graph", UUIDProvider::makeUUID_without_parent("~"), root_uuid_provider_.get());
    graph_new = std::dynamic_pointer_cast<Graph>(graph_new_handle->getNode().lock());
    graph_facade_new_ = std::make_shared<GraphFacade>(*executor, graph_new.get(), graph_new_handle.get());


    nh_old = node_factory.makeNode(node_->getType(), node_->getUUID(), graph_old.get());
    graph_old->addNode(nh_old);

    nh_new = node_factory.makeNode(type_new_, UUIDProvider::makeUUID_forced(graph_new, type_new_), graph_new.get());
    graph_new->addNode(nh_new);

    for(Slot* slot_original : node_->getSlots()) {
        for(ConnectionPtr connection : slot_original->getConnections()) {
            Slot* slot_old = nh_old->getSlot(slot_original->getUUID());
            updateConnection(slot_old, connection);
        }
    }
    for(InputPtr input_original : node_->getExternalInputs()) {
        for(ConnectionPtr connection : input_original->getConnections()) {
            Input* input_old = nh_old->getInput(input_original->getUUID());
            updateConnection(input_old, connection);
        }
    }


    for(Event* event_original : node_->getEvents()) {
        for(ConnectionPtr connection : event_original->getConnections()) {
            Event* event_old = nh_old->getEvent(event_original->getUUID());
            updateConnection(event_old, connection);
        }
    }
    for(OutputPtr output_original : node_->getExternalOutputs()) {
        for(ConnectionPtr connection : output_original->getConnections()) {
            Output* output_old = nh_old->getOutput(output_original->getUUID());
            updateConnection(output_old, connection);
        }
    }

    layout->addWidget(new QLabel("Current node:"));

    GraphView* view_old = new GraphView(graph_facade_old_, view_core_);
    view_old->overwriteStyleSheet(stylesheet);
    view_old->setMinimumSize(500, 350);
    view_old->setInteractive(false);
    layout->addWidget(view_old);

    layout->addWidget(new QLabel("New node:"));

    GraphView* view_new = new GraphView(graph_facade_new_, view_core_);
    view_new->overwriteStyleSheet(stylesheet);
    view_new->setMinimumSize(500, 350);
    layout->addWidget(view_new);

    QDialogButtonBox* buttons = new QDialogButtonBox;
    buttons->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    QObject::connect(buttons, SIGNAL(accepted()), this, SLOT(finish()));
    QObject::connect(buttons, SIGNAL(rejected()), this, SLOT(reject()));
}

void RewiringDialog::updateConnection(Input *input, const ConnectionPtr &connection)
{
    Output* out_original = connection->from();
    UUID uuid_old = graph_old->generateUUID(out_original->getUUID().id());

    UUID uuid_new = UUID::NONE;
    OutputPtr source_old = nullptr;
    OutputPtr source_new = nullptr;
    if(dynamic_cast<Event*>(out_original)) {
        source_old = graph_old->createInternalEvent(out_original->getType(), uuid_old, out_original->getLabel());
        source_new = graph_new->createInternalEvent(out_original->getType(), uuid_old, out_original->getLabel());
    } else {
        source_old = graph_old->createInternalOutput(out_original->getType(), uuid_old, out_original->getLabel(), out_original->isDynamic());
        source_new = graph_new->createInternalOutput(out_original->getType(), uuid_old, out_original->getLabel(), out_original->isDynamic());
    }

    graph_old->addConnection(DirectConnection::connect(source_old.get(), input));

    uuid_new = UUIDProvider::makeDerivedUUID_forced(nh_new->getUUID(), input->getUUID().id());
    if(Input* in_new = nh_new->getInput(uuid_new)) {
        graph_new->addConnection(DirectConnection::connect(source_new.get(), in_new));
    } else if(Slot* slot_new = nh_new->getSlot(uuid_new)) {
        graph_new->addConnection(DirectConnection::connect(source_new.get(), slot_new));
    }
}

void RewiringDialog::updateConnection(Output *output, const ConnectionPtr &connection)
{
    Input* in_original = connection->to();
    UUID uuid_old = graph_old->generateUUID(in_original->getUUID().id());

    UUID uuid_new = UUID::NONE;
    InputPtr sink_old = nullptr;
    InputPtr sink_new = nullptr;
    if(dynamic_cast<Slot*>(in_original)) {
        sink_old = graph_old->createInternalSlot(in_original->getType(), uuid_old, in_original->getLabel(), [](const TokenPtr&){});
        sink_new = graph_new->createInternalSlot(in_original->getType(), uuid_old, in_original->getLabel(), [](const TokenPtr&){});
    } else {
        sink_old = graph_old->createInternalInput(in_original->getType(), uuid_old, in_original->getLabel(), in_original->isDynamic(), in_original->isOptional());
        sink_new = graph_new->createInternalInput(in_original->getType(), uuid_old, in_original->getLabel(), in_original->isDynamic(), in_original->isOptional());
    }

    graph_old->addConnection(DirectConnection::connect(output, sink_old.get()));

    uuid_new = UUIDProvider::makeDerivedUUID_forced(nh_new->getUUID(), output->getUUID().id());
    if(Output* out_new = nh_new->getOutput(uuid_new)) {
        graph_new->addConnection(DirectConnection::connect(out_new, sink_new.get()));
    } else if(Event* event_new = nh_new->getEvent(uuid_new)) {
        graph_new->addConnection(DirectConnection::connect(event_new, sink_new.get()));
    }
}

void RewiringDialog::finish()
{
    if(true) {
        Q_EMIT reject();
    } else {
        Q_EMIT accept();
    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_rewiring_dialog.cpp"
