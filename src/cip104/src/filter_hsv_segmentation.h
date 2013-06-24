#ifndef FILTER_TOOL_DETECTION_H
#define FILTER_TOOL_DETECTION_H

/// COMPONENT
#include <vision_evaluator/filter.h>

/// SYSTEM
#include <QSlider>

namespace cip104
{

class HSVSegmentation : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    HSVSegmentation();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask);
    virtual void insert(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    void update();

private:
    QSlider* min_h;
    QSlider* min_s;
    QSlider* min_v;
    QSlider* max_h;
    QSlider* max_s;
    QSlider* max_v;


    struct State : public Memento {
        cv::Scalar min;
        cv::Scalar max;

        virtual void writeYaml(YAML::Emitter& out) const {
            out << YAML::Key << "min" << YAML::Value;
            out << YAML::BeginSeq << min[0] << min[1] << min[2] << YAML::EndSeq;
            out << YAML::Key << "max" << YAML::Value;
            out << YAML::BeginSeq << max[0] << max[1] << max[2] << YAML::EndSeq;
        }
        virtual void readYaml(const YAML::Node& node) {
            const YAML::Node& min_ = node["min"];
            const YAML::Node& max_ = node["max"];
            assert(min_.Type() == YAML::NodeType::Sequence);
            assert(max_.Type() == YAML::NodeType::Sequence);
            for(int i = 0; i < 3; ++i) {
                min_[i] >> min[i];
                max_[i] >> max[i];
            }
        }
    };

    State state;
};

}

#endif // FILTER_TOOL_DETECTION_H
