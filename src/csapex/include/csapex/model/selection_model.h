#ifndef SELECTION_MODEL_H
#define SELECTION_MODEL_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>

namespace csapex
{
class SelectionModel  : public QObject
{
    Q_OBJECT

public:
    SelectionModel(GraphPtr graph, WidgetController* widget_ctrl);

    virtual ~SelectionModel();

    void setCommandDispatcher(CommandDispatcher *dispatcher);

Q_SIGNALS:
    void selectionChanged();

protected:
    GraphPtr graph_;
    WidgetController* widget_ctrl_;
    CommandDispatcher *dispatcher_;
};
}

#endif // SELECTION_MODEL_H
