#ifndef ACTIVITY_LEGEND_H
#define ACTIVITY_LEGEND_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QFrame>
#include <QTableWidget>

namespace csapex
{

class CSAPEX_QT_EXPORT ActivityLegend : public QTableWidget, public Observer
{
    Q_OBJECT

public:
    ActivityLegend();

    void resizeToFit();

public Q_SLOTS:
    void startTrackingNode(NodeFacadePtr node);
    void stopTrackingNode(NodeFacadePtr node);

    void addNode(NodeFacade* node);
    void removeNode(NodeFacade* node);

    void emitSelection();

Q_SIGNALS:
    void nodeSelectionChanged(QList<NodeFacade*>);

    void nodeAdded(NodeFacade*);
    void nodeRemoved(NodeFacade*);


private:
    std::vector<NodeFacade*> rows_;
};

}

#endif // ACTIVITY_LEGEND_H

