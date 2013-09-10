#ifndef BOX_META_H
#define BOX_META_H

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/model/boxed_object.h>
#include <csapex/model/graph.h>
#include <csapex/model/template.h>

/// SYSTEM
#include <QLabel>

namespace csapex
{

class BoxGroup : public Box
{
public:
    typedef boost::shared_ptr<BoxGroup> Ptr;

public:
    static const QString MIME;

public:
    BoxGroup(BoxedObject::Ptr content, const std::string& uuid = "", QWidget* parent = 0);

    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

    void setTemplateName(const std::string& templ);

protected:
    Graph::Ptr sub_graph;

    Template::Ptr templ_;

    QLabel* icon_;
};

}

#endif // BOX_META_H
