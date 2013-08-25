#ifndef BOX_META_H
#define BOX_META_H

/// COMPONENT
#include <csapex/box.h>
#include <csapex/boxed_object.h>
#include <csapex/graph.h>

namespace csapex
{

class BoxGroup : public Box
{

public:
    static const QString MIME;

public:
    BoxGroup(BoxedObject::Ptr content, const std::string& uuid = "", QWidget* parent = 0);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragLeaveEvent(QDragLeaveEvent* e);
    void dropEvent(QDropEvent* e);

    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

protected:
    Graph::Ptr sub_graph;
};

}

#endif // BOX_META_H
