/// HEADER
#include <csapex/model/selection_model.h>

using namespace csapex;

SelectionModel::SelectionModel(GraphPtr graph, WidgetController* widget_ctrl)
    : graph_(graph), widget_ctrl_(widget_ctrl)
{
}

SelectionModel::~SelectionModel()
{
}

void SelectionModel::setCommandDispatcher(CommandDispatcher *dispatcher)
{
    dispatcher_ = dispatcher;
}
