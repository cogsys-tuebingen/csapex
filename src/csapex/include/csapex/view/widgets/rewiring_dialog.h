#ifndef REWIRING_DIALOG_H
#define REWIRING_DIALOG_H

/// PROJECT
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QDialog>

namespace csapex
{

class CsApexViewCore;
class Input;
class Output;

class RewiringDialog : public QDialog
{
    Q_OBJECT

public:
    RewiringDialog(GraphFacade* graph, NodeHandle *node, CsApexViewCore &view_core, QWidget *parent = 0, Qt::WindowFlags f = 0);
    ~RewiringDialog();

    void makeUI(const QString &stylesheet);
    std::string getName();

private Q_SLOTS:
    void finish();

private:
    void updateConnection(Input* input, const ConnectionPtr& connection);
    void updateConnection(Output* output, const ConnectionPtr& connection);

private:
    CsApexViewCore& view_core_;

    std::shared_ptr<UUIDProvider> root_uuid_provider_;

    ThreadPoolPtr executor;

    NodeHandlePtr graph_old_handle;
    NodeHandlePtr graph_new_handle;

    NodeHandlePtr nh_old;
    NodeHandlePtr nh_new;

    GraphFacade* graph_;
    NodeHandle* node_;

    GraphPtr graph_old;
    GraphFacadePtr graph_facade_old_;

    std::string type_new_;
    GraphPtr graph_new;
    GraphFacadePtr graph_facade_new_;
};

}


#endif // REWIRING_DIALOG_H
