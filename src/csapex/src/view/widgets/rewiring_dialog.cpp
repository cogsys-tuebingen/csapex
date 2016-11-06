/// HEADER
#include <csapex/view/widgets/rewiring_dialog.h>

/// COMPONENT
#include <csapex/view/designer/graph_view.h>
#include <csapex/core/csapex_core.h>
#include <csapex/model/subgraph_node.h>
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


RewiringDialog::RewiringDialog(NodeHandle* node, CsApexViewCore& view_core, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f),
      view_core_(view_core),

      core_temp_(std::make_shared<CsApexCore>(view_core.getCore())),
      temp_dispatcher_(std::make_shared<CommandDispatcher>(*core_temp_)),
      view_core_temp_(std::make_shared<CsApexViewCore>(view_core_, *core_temp_, temp_dispatcher_)),
      node_(node)
{
    root_uuid_provider_ = std::make_shared<UUIDProvider>();

    core_temp_->init();
}

RewiringDialog::~RewiringDialog()
{
    //    graph_facade_old_.reset();
    //    graph_facade_new_.reset();
    //    graph_old.reset();
    //    graph_new.reset();

    //    executor.reset();
}

std::string RewiringDialog::getType() const
{
    return type_new_;
}

std::vector<ConnectionInformation> RewiringDialog::getConnections(const UUID& new_node_uuid)
{
    for(SlotPtr slot_new : nh_new->getSlots()) {
        for(ConnectionPtr connection : slot_new->getConnections()) {
            UUID to_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, slot_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionInformation(new_target_uuid_to_old_uuid_.at(connection->from()->getUUID()),
                                                         to_uuid,
                                                         slot_new->getType(),
                                                         connection->isActive()));
        }
    }
    for(InputPtr input_new : nh_new->getExternalInputs()) {
        for(ConnectionPtr connection : input_new->getConnections()) {
            UUID to_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, input_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionInformation(new_target_uuid_to_old_uuid_.at(connection->from()->getUUID()),
                                                         to_uuid,
                                                         input_new->getType(),
                                                         connection->isActive()));
        }
    }


    for(EventPtr event_new : nh_new->getEvents()) {
        for(ConnectionPtr connection : event_new->getConnections()) {
            UUID from_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, event_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionInformation(from_uuid,
                                                         new_target_uuid_to_old_uuid_.at(connection->to()->getUUID()),
                                                         event_new->getType(),
                                                         connection->isActive()));
        }
    }
    for(OutputPtr output_new : nh_new->getExternalOutputs()) {
        for(ConnectionPtr connection : output_new->getConnections()) {
            UUID from_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, output_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionInformation(from_uuid,
                                                         new_target_uuid_to_old_uuid_.at(connection->to()->getUUID()),
                                                         output_new->getType(),
                                                         connection->isActive()));
        }
    }

    return connections_;
}

void RewiringDialog::makeUI(const QString& stylesheet)
{
    BoxDialog diag("Please enter the new node type.",
                   view_core_temp_->getCore().getNodeFactory(), *view_core_temp_->getNodeAdapterFactory(),
                   view_core_temp_->getCore().getSnippetFactory());
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


    NodeFactory& node_factory = view_core_temp_->getCore().getNodeFactory();
    executor = std::make_shared<ThreadPool>(view_core_temp_->getCore().getExceptionHandler(), false, false);

    graph_old_handle = node_factory.makeNode("csapex::Graph", UUIDProvider::makeUUID_without_parent("~"), root_uuid_provider_.get());
    apex_assert_hard(graph_old_handle);
    graph_old = std::dynamic_pointer_cast<SubgraphNode>(graph_old_handle->getNode().lock());
    apex_assert_hard(graph_old);
    graph_facade_old_ = std::make_shared<GraphFacade>(*executor, graph_old.get(), graph_old_handle.get());

    graph_facade_new_ = core_temp_->getRoot();
    graph_new = graph_facade_new_->getSubgraphNode();

    graph_new->removeInternalPorts();


    nh_old = node_factory.makeNode(node_->getType(), node_->getUUID(), graph_old.get());
    graph_old->addNode(nh_old);

    nh_new = node_factory.makeNode(type_new_, graph_new->generateUUID(type_new_), graph_new);
    graph_new->addNode(nh_new);


    for(SlotPtr slot_original : node_->getSlots()) {
        for(ConnectionPtr connection : slot_original->getConnections()) {
            SlotPtr slot_old = nh_old->getSlot(slot_original->getUUID());
            updateConnection(slot_old, connection);
        }
    }
    for(InputPtr input_original : node_->getExternalInputs()) {
        for(ConnectionPtr connection : input_original->getConnections()) {
            InputPtr input_old = nh_old->getInput(input_original->getUUID());
            updateConnection(input_old, connection);
        }
    }


    for(EventPtr event_original : node_->getEvents()) {
        for(ConnectionPtr connection : event_original->getConnections()) {
            EventPtr event_old = nh_old->getEvent(event_original->getUUID());
            updateConnection(event_old, connection);
        }
    }
    for(OutputPtr output_original : node_->getExternalOutputs()) {
        for(ConnectionPtr connection : output_original->getConnections()) {
            OutputPtr output_old = nh_old->getOutput(output_original->getUUID());
            updateConnection(output_old, connection);
        }
    }

    layout->addWidget(new QLabel("Current node:"));

    GraphView* view_old = new GraphView(graph_facade_old_, *view_core_temp_);
    view_old->overwriteStyleSheet(stylesheet);
    view_old->setMinimumSize(500, 350);
    view_old->setInteractive(false);
    layout->addWidget(view_old);

    layout->addWidget(new QLabel("New node:"));

    GraphView* view_new = new GraphView(graph_facade_new_, *view_core_temp_);
    view_new->overwriteStyleSheet(stylesheet);
    view_new->setMinimumSize(500, 350);
    layout->addWidget(view_new);

    QDialogButtonBox* buttons = new QDialogButtonBox;
    buttons->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    QObject::connect(buttons, SIGNAL(accepted()), this, SLOT(finish()));
    QObject::connect(buttons, SIGNAL(rejected()), this, SLOT(reject()));
}

void RewiringDialog::updateConnection(InputPtr input, const ConnectionPtr &connection)
{
    OutputPtr out_original = connection->from();
    UUID original_uuid = out_original->getUUID();
    UUID uuid_old;
    if(uuid_cache_.find(original_uuid) != uuid_cache_.end()) {
        uuid_old = uuid_cache_.at(original_uuid);
    } else {
        uuid_old = graph_old->generateUUID(original_uuid.id().getFullName());
        uuid_cache_[original_uuid] = uuid_old;
        new_target_uuid_to_old_uuid_[uuid_old] = original_uuid;
    }


    UUID uuid_new = UUID::NONE;
    OutputPtr source_old = graph_old->findConnectorNoThrow<Output>(uuid_old);
    OutputPtr source_new = graph_new->findConnectorNoThrow<Output>(uuid_old);
    if(std::dynamic_pointer_cast<Event>(out_original)) {
        if(!source_old) {
            source_old = graph_old->createInternalEvent(out_original->getType(), uuid_old, out_original->getLabel());
        }
        if(!source_new) {
            source_new = graph_new->createInternalEvent(out_original->getType(), uuid_old, out_original->getLabel());
        }
    } else {
        if(!source_old) {
            source_old = graph_old->createInternalOutput(out_original->getType(), uuid_old, out_original->getLabel());
        }
        if(!source_new) {
            source_new = graph_new->createInternalOutput(out_original->getType(), uuid_old, out_original->getLabel());
        }
    }

    graph_old->addConnection(DirectConnection::connect(source_old, input));

    uuid_new = UUIDProvider::makeDerivedUUID_forced(nh_new->getUUID(), input->getUUID().id().getFullName());

    ConnectionPtr c;
    if(InputPtr in_new = nh_new->getInput(uuid_new)) {
        c = DirectConnection::connect(source_new, in_new);
    } else if(SlotPtr slot_new = nh_new->getSlot(uuid_new)) {
        c = DirectConnection::connect(source_new, slot_new);
    }
    if(c) {
        c->setActive(connection->isActive());
        graph_new->addConnection(c);
    }
}

void RewiringDialog::updateConnection(OutputPtr output, const ConnectionPtr &connection)
{
    InputPtr in_original = connection->to();
    UUID uuid_old;
    UUID original_uuid = in_original->getUUID();
    if(uuid_cache_.find(original_uuid) != uuid_cache_.end()) {
        uuid_old = uuid_cache_.at(original_uuid);
    } else {
        uuid_old = graph_old->generateUUID(original_uuid.id().getFullName());
        uuid_cache_[original_uuid] = uuid_old;
        new_target_uuid_to_old_uuid_[uuid_old] = original_uuid;
    }

    UUID uuid_new = UUID::NONE;
    InputPtr sink_old = graph_old->findConnectorNoThrow<Input>(uuid_old);
    InputPtr sink_new = graph_new->findConnectorNoThrow<Input>(uuid_old);
    if(std::dynamic_pointer_cast<Slot>(in_original)) {
        if(!sink_old) {
            sink_old = graph_old->createInternalSlot(in_original->getType(), uuid_old, in_original->getLabel(), [](const TokenPtr&){});
        }
        if(!sink_new) {
            sink_new = graph_new->createInternalSlot(in_original->getType(), uuid_old, in_original->getLabel(), [](const TokenPtr&){});
        }
    } else {
        if(!sink_old) {
            sink_old = graph_old->createInternalInput(in_original->getType(), uuid_old, in_original->getLabel(), in_original->isOptional());
        }
        if(!sink_new) {
            sink_new = graph_new->createInternalInput(in_original->getType(), uuid_old, in_original->getLabel(), in_original->isOptional());
        }
    }

    graph_old->addConnection(DirectConnection::connect(output, sink_old));

    uuid_new = UUIDProvider::makeDerivedUUID_forced(nh_new->getUUID(), output->getUUID().id().getFullName());

    ConnectionPtr c;
    if(OutputPtr out_new = nh_new->getOutput(uuid_new)) {
        c = DirectConnection::connect(out_new, sink_new);
    } else if(EventPtr event_new = nh_new->getEvent(uuid_new)) {
        c = DirectConnection::connect(event_new, sink_new);
    }

    if(c) {
        c->setActive(connection->isActive());
        graph_new->addConnection(c);
    }
}

void RewiringDialog::finish()
{
    Q_EMIT accept();
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_rewiring_dialog.cpp"
