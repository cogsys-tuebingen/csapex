#ifndef REWIRING_DIALOG_H
#define REWIRING_DIALOG_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/connection_information.h>

/// SYSTEM
#include <QDialog>
#include <unordered_map>

namespace csapex
{

class CsApexViewCore;
class CommandDispatcher;
class CsApexCore;

class CSAPEX_QT_EXPORT RewiringDialog : public QDialog
{
    Q_OBJECT

public:
    RewiringDialog(NodeHandle *node, CsApexViewCore &view_core, QWidget *parent = 0, Qt::WindowFlags f = 0);
    ~RewiringDialog();

    void makeUI(const QString &stylesheet);

    std::string getType() const;

    std::vector<ConnectionInformation> getConnections(const UUID &new_node_uuid);

private Q_SLOTS:
    void finish();

private:
    void updateConnection(InputPtr input, const ConnectionPtr& connection);
    void updateConnection(OutputPtr output, const ConnectionPtr& connection);

private:
    CsApexViewCore& view_core_;

    std::shared_ptr<CsApexCore> core_temp_;
    std::shared_ptr<CommandDispatcher> temp_dispatcher_;
    std::shared_ptr<CsApexViewCore> view_core_temp_;

    std::shared_ptr<UUIDProvider> root_uuid_provider_;

    ThreadPoolPtr executor;

    NodeHandlePtr graph_old_handle;
    NodeHandlePtr graph_new_handle;

    NodeHandlePtr nh_old;
    NodeHandlePtr nh_new;

    NodeHandle* node_;

    SubgraphNodePtr graph_old;
    GraphFacadePtr graph_facade_old_;

    std::string type_new_;
    SubgraphNode* graph_new;
    GraphFacadePtr graph_facade_new_;

    std::vector<ConnectionInformation> connections_;

    std::unordered_map<UUID, UUID, UUID::Hasher> new_target_uuid_to_old_uuid_;
    std::unordered_map<UUID, UUID, UUID::Hasher> uuid_cache_;
};

}


#endif // REWIRING_DIALOG_H
