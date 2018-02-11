/// HEADER
#include <csapex/view/widgets/rewiring_dialog.h>

/// COMPONENT
#include <csapex/factory/node_factory_impl.h>
#include <csapex/model/connection.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/view/csapex_view_core_impl.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/widgets/box_dialog.h>

/// SYSTEM
#include <QLabel>
#include <QKeyEvent>
#include <QListView>
#include <QVBoxLayout>
#include <QDialogButtonBox>

using namespace csapex;


RewiringDialog::RewiringDialog(NodeFacade* node, CsApexViewCore& view_core, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f),
      view_core_(view_core),

      view_core_old_(std::make_shared<CsApexViewCoreImplementation>(view_core_, view_core_.getExceptionHandler())),
      view_core_new_(std::make_shared<CsApexViewCoreImplementation>(view_core_, view_core_.getExceptionHandler())),
      node_facade_(node)
{
    root_uuid_provider_ = std::make_shared<UUIDProvider>();
}

RewiringDialog::~RewiringDialog()
{
}

std::string RewiringDialog::getType() const
{
    return type_new_;
}

std::vector<ConnectionDescription> RewiringDialog::getConnections(const UUID& new_node_uuid)
{
    for(SlotPtr slot_new : nf_new->getNodeHandle()->getSlots()) {
        for(ConnectionPtr connection : slot_new->getConnections()) {
            UUID to_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, slot_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionDescription(new_target_uuid_to_old_uuid_.at(connection->from()->getUUID()),
                                                         to_uuid,
                                                         slot_new->getType(),
                                                         connection->id(),
                                                         connection->isActive(),
                                                         connection->getFulcrumsCopy()));
        }
    }
    for(InputPtr input_new : nf_new->getNodeHandle()->getExternalInputs()) {
        for(ConnectionPtr connection : input_new->getConnections()) {
            UUID to_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, input_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionDescription(new_target_uuid_to_old_uuid_.at(connection->from()->getUUID()),
                                                         to_uuid,
                                                         input_new->getType(),
                                                         connection->id(),
                                                         connection->isActive(),
                                                         connection->getFulcrumsCopy()));
        }
    }


    for(EventPtr event_new : nf_new->getNodeHandle()->getEvents()) {
        for(ConnectionPtr connection : event_new->getConnections()) {
            UUID from_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, event_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionDescription(from_uuid,
                                                         new_target_uuid_to_old_uuid_.at(connection->to()->getUUID()),
                                                         event_new->getType(),
                                                         connection->id(),
                                                         connection->isActive(),
                                                         connection->getFulcrumsCopy()));
        }
    }
    for(OutputPtr output_new : nf_new->getNodeHandle()->getExternalOutputs()) {
        for(ConnectionPtr connection : output_new->getConnections()) {
            UUID from_uuid = UUIDProvider::makeDerivedUUID_forced(new_node_uuid, output_new->getUUID().id().getFullName());
            connections_.push_back(ConnectionDescription(from_uuid,
                                                         new_target_uuid_to_old_uuid_.at(connection->to()->getUUID()),
                                                         output_new->getType(),
                                                         connection->id(),
                                                         connection->isActive(),
                                                         connection->getFulcrumsCopy()));
        }
    }

    return connections_;
}

void RewiringDialog::makeUI(const QString& stylesheet)
{
    BoxDialog diag("Please enter the new node type.",
                   *view_core_.getNodeFactory(),
                   *view_core_.getNodeAdapterFactory(),
                   view_core_.getSnippetFactory());
    diag.setWindowTitle("Select new node type.");

    int r = diag.exec();

    if(!r) {
        return;
    }

    type_new_ = diag.getName();

    createGraphs(type_new_);
    createConnections();


    createUI(stylesheet);
}

void RewiringDialog::createGraphs(const std::string& type)
{
    NodeFactoryImplementationPtr nfl = std::dynamic_pointer_cast<NodeFactoryImplementation>(view_core_.getNodeFactory());
    if(!nfl) {
        throw std::runtime_error("currently not supported");
    }
    NodeFactoryImplementation& node_factory = *nfl;

    // old graph
    graph_facade_old_ = view_core_old_->getLocalRoot();
    apex_assert_hard(graph_facade_old_);
    NodeFacadeImplementationPtr gnf_old = std::dynamic_pointer_cast<NodeFacadeImplementation>(graph_facade_old_->getNodeFacade());
    apex_assert_hard(gnf_old);
    graph_node_old = std::dynamic_pointer_cast<SubgraphNode>(
                gnf_old->getNodeHandle()->getNode().lock());
    apex_assert_hard(graph_node_old);
    graph_old = graph_facade_old_->getLocalGraph();
    apex_assert_hard(graph_old);

    nf_old = node_factory.makeNode(node_facade_->getType(), node_facade_->getUUID(), graph_old);
    nf_old->setNodeState(node_facade_->getNodeStateCopy());
    graph_old->addNode(nf_old);


    // new graph
    graph_facade_new_ = view_core_new_->getLocalRoot();
    apex_assert_hard(graph_facade_new_);
    NodeFacadeImplementationPtr gnf_new = std::dynamic_pointer_cast<NodeFacadeImplementation>(graph_facade_new_->getNodeFacade());
    apex_assert_hard(gnf_new);
    graph_node_new = std::dynamic_pointer_cast<SubgraphNode>(
                gnf_new->getNodeHandle()->getNode().lock());
    apex_assert_hard(graph_node_new);
    graph_new = graph_facade_new_->getLocalGraph();
    apex_assert_hard(graph_new);

    nf_new = node_factory.makeNode(type, graph_new->generateUUID(type), graph_new);
    graph_new->addNode(nf_new);
}

void RewiringDialog::createConnections()
{
    for(const ConnectorDescription& slot_original : node_facade_->getSlots()) {
        for(const ConnectorDescription::Target& target : slot_original.targets) {
            SlotPtr slot_old = nf_old->getNodeHandle()->getSlot(slot_original.id);
            apex_assert_hard(slot_old);
            updateConnection(slot_old, slot_original, target.auuid, target.active);
        }
    }
    for(const ConnectorDescription& input_original : node_facade_->getExternalInputs()) {
        for(const ConnectorDescription::Target& target : input_original.targets) {
            InputPtr input_old = nf_old->getNodeHandle()->getInput(input_original.id);
            apex_assert_hard(input_old);
            updateConnection(input_old, input_original, target.auuid, target.active);
        }
    }


    for(const ConnectorDescription& event_original : node_facade_->getEvents()) {
        for(const ConnectorDescription::Target& target : event_original.targets) {
            EventPtr event_old = nf_old->getNodeHandle()->getEvent(event_original.id);
            apex_assert_hard(event_old);
            updateConnection(event_old, event_original, target.auuid, target.active);
        }
    }
    for(const ConnectorDescription& output_original : node_facade_->getExternalOutputs()) {
        for(const ConnectorDescription::Target& target : output_original.targets) {
            OutputPtr output_old = nf_old->getNodeHandle()->getOutput(output_original.id);
            updateConnection(output_old, output_original, target.auuid, target.active);
        }
    }
}

void RewiringDialog::createUI(const QString& stylesheet)
{
    setStyleSheet(stylesheet);
    setWindowIcon(QIcon(":/pencil.png"));
    setWindowTitle(QString("Change Node to ") + QString::fromStdString(type_new_));

    setFocusPolicy(Qt::StrongFocus);
    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    layout->addWidget(new QLabel("Current node:"));

    GraphView* view_old = new GraphView(graph_facade_old_, *view_core_old_);
    view_old->overwriteStyleSheet(stylesheet);
    view_old->setMinimumSize(500, 350);
    view_old->setInteractive(false);
    layout->addWidget(view_old);

    layout->addWidget(new QLabel("New node:"));

    GraphView* view_new = new GraphView(graph_facade_new_, *view_core_new_);
    view_new->overwriteStyleSheet(stylesheet);
    view_new->setMinimumSize(500, 350);
    layout->addWidget(view_new);

    QDialogButtonBox* buttons = new QDialogButtonBox;
    buttons->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);

    QObject::connect(buttons, SIGNAL(accepted()), this, SLOT(finish()));
    QObject::connect(buttons, SIGNAL(rejected()), this, SLOT(reject()));
}


void RewiringDialog::updateConnection(InputPtr input, const ConnectorDescription &connector, const AUUID& target, bool active)
{
    UUID original_uuid = target;
    UUID uuid_old;
    if(uuid_cache_.find(original_uuid) != uuid_cache_.end()) {
        uuid_old = uuid_cache_.at(original_uuid);
    } else {
        uuid_old = graph_old->generateUUID(original_uuid.id().type());
        uuid_cache_[original_uuid] = uuid_old;
        new_target_uuid_to_old_uuid_[uuid_old] = original_uuid;
    }

    UUID uuid_new = UUID::NONE;
    OutputPtr source_old = graph_old->findTypedConnectorNoThrow<Output>(uuid_old);
    OutputPtr source_new = graph_new->findTypedConnectorNoThrow<Output>(uuid_old);
    if(target.type() == "event" || target.type() == "relayevent") {
        if(!source_old) {
            source_old = graph_node_old->createInternalEvent(connector.token_type, uuid_old, connector.label);
        }
        if(!source_new) {
            source_new = graph_node_new->createInternalEvent(connector.token_type, uuid_old, connector.label);
        }
    } else {
        if(!source_old) {
            source_old = graph_node_old->createInternalOutput(connector.token_type, uuid_old, connector.label);
        }
        if(!source_new) {
            source_new = graph_node_new->createInternalOutput(connector.token_type, uuid_old, connector.label);
        }
    }

    graph_old->addConnection(DirectConnection::connect(source_old, input));

    uuid_new = UUIDProvider::makeDerivedUUID_forced(nf_new->getUUID(), input->getUUID().id().getFullName());

    ConnectionPtr c;
    if(InputPtr in_new = nf_new->getNodeHandle()->getInput(uuid_new)) {
        if(!Connection::isCompatibleWith(source_new.get(), in_new.get())) {
            return;
        }
        c = DirectConnection::connect(source_new, in_new);
    } else if(SlotPtr slot_new = nf_new->getNodeHandle()->getSlot(uuid_new)) {
        if(!Connection::isCompatibleWith(source_new.get(), slot_new.get())) {
            return;
        }
        c = DirectConnection::connect(source_new, slot_new);
    }
    if(c) {
        c->setActive(active);
        graph_new->addConnection(c);
    }
}

void RewiringDialog::updateConnection(OutputPtr output, const ConnectorDescription &connector, const AUUID &target, bool active)
{
    UUID uuid_old;
    UUID original_uuid = target;
    if(uuid_cache_.find(original_uuid) != uuid_cache_.end()) {
        uuid_old = uuid_cache_.at(original_uuid);
    } else {
        uuid_old = graph_old->generateUUID(original_uuid.id().type());
        uuid_cache_[original_uuid] = uuid_old;
        new_target_uuid_to_old_uuid_[uuid_old] = original_uuid;
    }

    UUID uuid_new = UUID::NONE;
    InputPtr sink_old = graph_old->findTypedConnectorNoThrow<Input>(uuid_old);
    InputPtr sink_new = graph_new->findTypedConnectorNoThrow<Input>(uuid_old);
    if(target.type() == "slot" || target.type() == "relayslot") {
        if(!sink_old) {
            sink_old = graph_node_old->createInternalSlot(connector.token_type, uuid_old, connector.label, [](const TokenPtr&){});
        }
        if(!sink_new) {
            sink_new = graph_node_new->createInternalSlot(connector.token_type, uuid_old, connector.label, [](const TokenPtr&){});
        }
    } else {
        if(!sink_old) {
            sink_old = graph_node_old->createInternalInput(connector.token_type, uuid_old, connector.label, connector.optional);
        }
        if(!sink_new) {
            sink_new = graph_node_new->createInternalInput(connector.token_type, uuid_old, connector.label, connector.optional);
        }
    }

    graph_old->addConnection(DirectConnection::connect(output, sink_old));

    uuid_new = UUIDProvider::makeDerivedUUID_forced(nf_new->getUUID(), output->getUUID().id().getFullName());

    ConnectionPtr c;
    if(OutputPtr out_new = nf_new->getNodeHandle()->getOutput(uuid_new)) {
        c = DirectConnection::connect(out_new, sink_new);
    } else if(EventPtr event_new = nf_new->getNodeHandle()->getEvent(uuid_new)) {
        c = DirectConnection::connect(event_new, sink_new);
    }

    if(c) {
        c->setActive(active);
        graph_new->addConnection(c);
    }
}

void RewiringDialog::finish()
{
    Q_EMIT accept();
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_rewiring_dialog.cpp"
