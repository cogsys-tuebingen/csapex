#ifndef FILTER_MERGER_H
#define FILTER_MERGER_H

/// COMPONENT
#include <vision_evaluator/filter.h>

class QSpinBox;

namespace vision_evaluator {

const static int MERGER_INPUT_MAX = 10;
/**
 * @brief The Merger class can be used to merge a certain amount of
 *        images.
 */
class Merger : public BoxedObject
{
    Q_OBJECT

public:
    /**
     * @brief Merger Constructor.
     */
    Merger();

    /**
     * @brief See base class documentation.
     */
    virtual void fill(QBoxLayout *layout);

    /// MEMENTO
    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

private Q_SLOTS:
    void messageArrived(ConnectorIn *source);
    void updateInputs(int value);

private:
    ConnectorOut *output_;
    std::map<ConnectorIn*,bool> input_arrivals_;
    QSpinBox                    *input_count_;

    void collectMessage(std::vector<cv::Mat> &messages);
    void updateArrivals(ConnectorIn *input);
    bool gotAllArrivals();
    void resetInputArrivals();

    /// MEMENTO
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Emitter &out) const;

    public:
        int     input_count;

    };

    State state_;

};
}
#endif // FILTER_MERGER_H
