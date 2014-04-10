#ifndef WIDGET_CONTROLLER_H
#define WIDGET_CONTROLLER_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>

namespace csapex
{

class WidgetController : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<WidgetController> Ptr;

public:
    WidgetController(GraphPtr graph);

    Box* getBox(const UUID& node_id);

    GraphPtr getGraph();

    void setDesigner(Designer* designer);

public Q_SLOTS:
    void nodeAdded(NodePtr node);
    void nodeRemoved(NodePtr node);

private:
    GraphPtr graph_;

    Designer* designer_;
};

}

#endif // WIDGET_CONTROLLER_H
