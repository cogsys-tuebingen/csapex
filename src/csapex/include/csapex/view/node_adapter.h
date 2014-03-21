#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/param_fwd.h>
#include <utils_param/parameter.h>

/// SYSTEM
#include <QLayout>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <vector>
#include <QComboBox>

class QListView;

namespace csapex
{

/// TODO: move to default node adapter!!!
class NodeAdapterBridge : public QObject
{
    Q_OBJECT

public:
    NodeAdapterBridge(NodeAdapter* parent);

public Q_SLOTS:
    void modelChangedEvent();
    void rebuildEvent();

Q_SIGNALS:
    void guiChanged();
    void rebuild();

public:
    void triggerGuiChanged();
    void triggerRebuild();

private:
    NodeAdapter* parent_;
};

class NodeAdapter : public param::Parameter::access
{
    friend class NodeAdapterBridge;

public:
    typedef boost::shared_ptr<NodeAdapter> Ptr;

protected:
    NodeAdapter(Node* adaptee);

public:
    virtual ~NodeAdapter();

//    void setNode(Node* node);
    Node* getNode();

    void setupUiAgain();
    void doSetupUi(QBoxLayout* layout);

    virtual void modelChangedEvent();
    virtual void updateDynamicGui(QBoxLayout* layout);

protected:
    virtual void setupUi(QBoxLayout* layout) = 0;

    void guiChanged();

public:
    NodeAdapterBridge bridge;

protected:
    QBoxLayout *layout_;
    bool is_gui_setup_;

    std::string current_name_;
    std::string current_display_name_;
    QBoxLayout* current_layout_;

    Node* node_;
};

}

#endif // NODE_ADAPTER_H
