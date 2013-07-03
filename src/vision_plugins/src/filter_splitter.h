#ifndef FILTER_SPLITTER_H
#define FILTER_SPLITTER_H

/// COMPONENT
#include <vision_evaluator/filter.h>

namespace vision_evaluator {
class Splitter : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    Splitter();

    virtual void fill(QBoxLayout* layout);


    void setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

private Q_SLOTS:
    void messageArrived(ConnectorIn *source);

private:
    ConnectorIn *input_;

    void updateOutputs();

    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["channel_count"] >> channel_count_;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "channel_count" << YAML::Value << channel_count_;
        }

    public:
        int channel_count_;
    };

    State state_;

};
}
#endif // FILTER_SPLITTER_H
