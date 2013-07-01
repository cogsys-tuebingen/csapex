#ifndef FILTER_HISTOGRAM_H
#define FILTER_HISTOGRAM_H

class QDoubleSlider;
class QSpinBox;

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
    Memento::Ptr getState();

    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);

private:
    /// connectors
    ConnectorIn   *input_;
    ConnectorOut  *output_histogram_;

    /// layout and computation
    int            channel_count_;
    QBoxLayout    *layout_;
    QDoubleSlider *slide_zoom_;
    QWidget       *container_zoom_;
    QWidget       *container_bin_counts_;

    std::vector<QSpinBox*>  bin_counts_;
    std::vector<cv::Scalar> colors_;

    void prepare(cv::Mat &bins, cv::Mat &ranges);
    void updateSliders();
    cv::Scalar randomColor();

    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["ch1_min"] >> ch1_min;
            node["ch1_max"] >> ch1_max;
            node["ch2_min"] >> ch2_min;
            node["ch2_max"] >> ch2_max;
            node["ch3_min"] >> ch3_min;
            node["ch3_max"] >> ch3_max;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "ch1_min" << YAML::Value << ch1_min;
            out << YAML::Key << "ch1_max" << YAML::Value << ch1_max;
            out << YAML::Key << "ch2_min" << YAML::Value << ch2_min;
            out << YAML::Key << "ch2_max" << YAML::Value << ch2_max;
            out << YAML::Key << "ch3_min" << YAML::Value << ch3_min;
            out << YAML::Key << "ch3_max" << YAML::Value << ch3_max;
        }

    public:
        double ch1_min;
        double ch1_max;
        double ch2_min;
        double ch2_max;
        double ch3_min;
        double ch3_max;


    };

    State state_;

};
}
#endif // FILTER_HISTOGRAM_H
