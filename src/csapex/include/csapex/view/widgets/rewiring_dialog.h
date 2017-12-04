#ifndef REWIRING_DIALOG_H
#define REWIRING_DIALOG_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/connection_description.h>
#include <csapex/utility/utility_fwd.h>

/// SYSTEM
#include <QDialog>
#include <unordered_map>

namespace csapex
{

class CsApexViewCore;
class CsApexViewCoreImplementation;
class CommandExecutor;
class CsApexCore;

class CSAPEX_QT_EXPORT RewiringDialog : public QDialog
{
    Q_OBJECT

public:
    RewiringDialog(NodeFacade *node, CsApexViewCore &view_core, QWidget *parent = 0, Qt::WindowFlags f = 0);
    ~RewiringDialog();

    void makeUI(const QString &stylesheet);

    std::string getType() const;

    std::vector<ConnectionDescription> getConnections(const UUID &new_node_uuid);



private Q_SLOTS:
    void finish();


private:
    void createGraphs(const std::string &type);

    void createConnections();
    void updateConnection(InputPtr input, const ConnectorDescription &connector, const AUUID &connected, bool active);
    void updateConnection(OutputPtr output, const ConnectorDescription &connector, const AUUID& connected, bool active);

    void createUI(const QString& stylesheet);

private:
    CsApexViewCore& view_core_;

    std::shared_ptr<CsApexViewCoreImplementation> view_core_old_;
    std::shared_ptr<CsApexViewCoreImplementation> view_core_new_;

    UUIDProviderPtr root_uuid_provider_;

    NodeFacadeImplementationPtr nf_old;
    NodeFacadeImplementationPtr nf_new;

    NodeFacade* node_facade_;

    SubgraphNodePtr graph_node_old;
    GraphImplementationPtr graph_old;
    GraphFacadeImplementationPtr graph_facade_old_;

    std::string type_new_;
    SubgraphNodePtr graph_node_new;
    GraphImplementationPtr graph_new;
    GraphFacadeImplementationPtr graph_facade_new_;

    std::vector<ConnectionDescription> connections_;

    std::unordered_map<UUID, UUID, UUID::Hasher> new_target_uuid_to_old_uuid_;
    std::unordered_map<UUID, UUID, UUID::Hasher> uuid_cache_;
};

}


#endif // REWIRING_DIALOG_H
