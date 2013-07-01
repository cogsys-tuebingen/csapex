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

    std::vector<QSlider*>  bin_counts_;
    std::vector<cv::Scalar> colors_;

    void prepare(cv::Mat &bins, cv::Mat &ranges);
    void updateSliders();
    cv::Scalar randomColor();

    /// memento
    void updateState();
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            const YAML::Node &values = node["bin_counts"];
            for(YAML::Iterator it = values.begin() ; it != values.end() ; it++) {
                int value;
                *it >> value;
                bin_counts.push_back(value);
            }
            node["channel_count"] >> channel_count;
            node["zoom"] >> zoom;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "bin_counts" << YAML::Value << YAML::BeginSeq;
            for(std::vector<int>::const_iterator it = bin_counts.begin() ; it != bin_counts.end() ; it++) {
                out << *it;
            }
            out << YAML::EndSeq;
            out << YAML::Key << "channel_count" << YAML::Value << channel_count;
            out << YAML::Key << "zoom" << YAML::Value << zoom;
        }

    public:
        std::vector<int> bin_counts;
        int     channel_count;
        double  zoom;

    };

    State state_;

};
}
#endif // FILTER_HISTOGRAM_H
