#ifndef RESIZABLE_NODE_ADAPTER_H
#define RESIZABLE_NODE_ADAPTER_H

/// PROJECT
#include <csapex/view/node/default_node_adapter.h>

namespace csapex
{
class CSAPEX_QT_EXPORT ResizableNodeAdapter : public DefaultNodeAdapter
{
public:
    ResizableNodeAdapter(NodeFacadePtr worker, NodeBox* parent);

    void readLegacyYaml(const YAML::Node& node) override;
    bool isResizable() const override;

    void setupUi(QBoxLayout* layout) override;

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

}  // namespace csapex

#endif  // RESIZABLE_NODE_ADAPTER_H
