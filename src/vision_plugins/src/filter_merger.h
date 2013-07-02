#ifndef FILTER_MERGER_H
#define FILTER_MERGER_H

/// COMPONENT
#include <vision_evaluator/filter.h>

class QSpinBox;

namespace vision_evaluator {

const static int MERGER_INPUT_MAX = 10;

class Merger : public BoxedObject
{
    Q_OBJECT

public:
    Merger();

    virtual void fill(QBoxLayout *layout);

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
};
}
#endif // FILTER_MERGER_H
