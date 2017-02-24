#ifndef RESIZABLE_NODE_ADAPTER_H
#define RESIZABLE_NODE_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

namespace csapex
{

class CSAPEX_QT_EXPORT ResizableNodeAdapter : public DefaultNodeAdapter
{
public:
    ResizableNodeAdapter(NodeHandleWeakPtr worker, NodeBox *parent);

    virtual void readLegacyYaml(const YAML::Node& node) override;
    virtual bool isResizable() const override;

    virtual void setupUi(QBoxLayout* layout) override;

protected:
    virtual void resize(const QSize& size) = 0;
    void doResize();

protected:
    void setSize(int width, int height);
    void readState();

private:
    QSize size_;
    bool initialized_;
};

}

#endif // RESIZABLE_NODE_ADAPTER_H
