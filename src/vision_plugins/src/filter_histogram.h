#ifndef FILTER_HISTOGRAM_H
#define FILTER_HISTOGRAM_H

class QDoubleSlider;
class QSpinBox;
class QSlider;

/// COMPONENT
#include <vision_evaluator/filter.h>


namespace vision_evaluator {
/// constants
static int HISTOGRAM_BINS_MAX = 512;
static int HISTOGRAM_BINS_STD = 256;

class Histogram : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    Histogram();

    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);
    void updateDynamicGui(QBoxLayout *layout);

private:
    /// connectors
    ConnectorIn   *input_;
    ConnectorOut  *output_histogram_;

    /// layout and computation
    int            channel_count_;
    QDoubleSlider *slide_zoom_;
    QWidget       *container_zoom_;
    QWidget       *container_bin_counts_;

    std::vector<QSlider*>  bin_counts_;
    std::vector<cv::Scalar> colors_;

    void prepare(cv::Mat &bins, cv::Mat &ranges);
    cv::Scalar randomColor();

    /// memento
    void updateState();
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Emitter &out) const;
    public:
        std::vector<int> bin_counts;
        int     channel_count;
        double  zoom;

    };

    State state_;

};
}
#endif // FILTER_HISTOGRAM_H
