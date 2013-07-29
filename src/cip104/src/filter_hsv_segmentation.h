#ifndef FILTER_TOOL_DETECTION_H
#define FILTER_TOOL_DETECTION_H

/// COMPONENT
#include <vision_evaluator/filter.h>

/// SYSTEM
#include <qxtspanslider.h>

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
    QxtSpanSlider* sliderHue;
    QxtSpanSlider* sliderSat;
    QxtSpanSlider* sliderVal;

    static QxtSpanSlider* makeSpanSlider(QBoxLayout* layout, const std::string& name, int lower, int upper, int min, int max);

    struct State : public Memento {
        cv::Scalar min;
        cv::Scalar max;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // FILTER_TOOL_DETECTION_H
