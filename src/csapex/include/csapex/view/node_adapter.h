#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// SYSTEM
#include <QLayout>
#include <boost/shared_ptr.hpp>

namespace csapex
{

class NodeAdapterBridge : public QObject
{
    Q_OBJECT

Q_SIGNALS:
    void guiChanged();

public:
    void triggerGuiChanged();

};

class NodeAdapter
{
public:
    typedef boost::shared_ptr<NodeAdapter> Ptr;

public:
    virtual ~NodeAdapter();

    virtual void fill(QBoxLayout* layout);
    virtual void updateDynamicGui(QBoxLayout* layout);

protected:
    void guiChanged();

public:
    NodeAdapterBridge bridge;
};

}

#endif // NODE_ADAPTER_H
