#ifndef CONNECTION_SELECTION_MODEL_H
#define CONNECTION_SELECTION_MODEL_H

/// COMPONENT
#include <csapex/model/selection_model.h>

namespace csapex
{

class ConnectionSelectionModel : public SelectionModel
{
    Q_OBJECT

public:
    ConnectionSelectionModel(GraphPtr graph, WidgetController* widget_ctrl);

    void handleSelection(int id, bool add);
    CommandPtr deleteSelectedCommand();

    void select(int id, bool add = false);
    int countSelected();

public Q_SLOTS:
    void clearSelection();

private:
    void deselectConnectionById(int id);

    bool isConnectionWithIdSelected(int id);
};

}

#endif // CONNECTION_SELECTION_MODEL_H
