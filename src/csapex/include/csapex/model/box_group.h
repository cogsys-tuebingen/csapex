#ifndef BOX_META_H
#define BOX_META_H

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/model/boxed_object.h>
#include <csapex/model/template.h>
#include <csapex/command/dispatcher.h>

/// SYSTEM
#include <QLabel>

namespace csapex
{

class BoxGroup : public Box
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<BoxGroup> Ptr;

public:
    static const QString MIME;

public:
    BoxGroup(const std::string& type, const std::string& uuid = "", QWidget* parent = 0);


    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

    virtual void init(const QPoint& pos);

Q_SIGNALS:
    void open_sub_graph(BoxGroup*);

public Q_SLOTS:
    void saveAsTemplate();

protected:
    bool eventFilter(QObject*, QEvent*);
    virtual void fillContextMenu(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);

protected:
    CommandDispatcher cmd_dispatcher;

    Template::Ptr templ_;

    QLabel* icon_;
};

}

#endif // BOX_META_H
