#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// COMPONENT
#include <csapex/command/command_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/model/observer.h>
#include <csapex_qt_export.h>
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QLayout>
#include <memory>
#include <csapex/utility/slim_signal.hpp>
#include <vector>

class QListView;

namespace YAML {
class Node;
}

namespace csapex
{
class CSAPEX_QT_EXPORT NodeAdapter : public Observer
{
public:
    typedef std::shared_ptr<NodeAdapter> Ptr;

protected:
    NodeAdapter(NodeFacadePtr adaptee, NodeBox* parent);

public:
    virtual ~NodeAdapter();

    void doSetupUi(QBoxLayout* layout);

    virtual void stop();

    virtual GenericStatePtr getState() const;
    virtual void setParameterState(GenericStatePtr memento);

    virtual void readLegacyYaml(const YAML::Node& node);

    virtual bool isResizable() const;
    virtual void setManualResize(bool manual);

public:
    csapex::slim_signal::Signal<void(const CommandPtr&)> executeCommand;

protected:
    virtual void setupUi(QBoxLayout* layout) = 0;
    void invalidate();

protected:
    QBoxLayout *layout_;
    bool is_gui_setup_;

    std::string current_name_;
    std::string current_display_name_;
    QBoxLayout* current_layout_;

    NodeFacadeWeakPtr node_;
    NodeBox* parent_;
};

}

#endif // NODE_ADAPTER_H
