#ifndef GLOBAL_OPTION_PANEL_H
#define GLOBAL_OPTION_PANEL_H

/// COMPONENT
#include "global_option.h"

/// PROJECT
#include <designer/boxed_object.h>

namespace vision_evaluator
{

class GlobalOptionPanel : public BoxedObject
{
    Q_OBJECT

public:
    GlobalOptionPanel();
    ~GlobalOptionPanel();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    virtual bool canBeDisabled() const;

    virtual void messageArrived(ConnectorIn* source);

private:
    void ensureLoaded();

    struct State : public Memento
    {
        typedef boost::shared_ptr<State> Ptr;

        State(GlobalOptionPanel* parent);

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);

        std::vector<Memento::Ptr> states;

        GlobalOptionPanel* parent;
    };

    State state;

    std::vector<GlobalOption::Ptr> options;
};

}

#endif // GLOBAL_OPTION_PANEL_H
