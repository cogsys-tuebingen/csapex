#ifndef FILTER_TOOL_DETECTION_H
#define FILTER_TOOL_DETECTION_H

/// COMPONENT
#include <csapex_vision/filter.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <qxtspanslider.h>

namespace csapex
{

class Segmentation : public csapex::BoxedObject
{
    Q_OBJECT

public:
    Segmentation();

public:
    virtual void allConnectorsArrived();
    virtual void fill(QBoxLayout *layout);
    virtual void updateDynamicGui(QBoxLayout *layout);

    virtual csapex::Memento::Ptr getState() const;
    virtual void setState(csapex::Memento::Ptr memento);

public Q_SLOTS:
    void update();

private:
    std::vector<QxtSpanSlider*> sliders;

    static QxtSpanSlider* makeSpanSlider(QBoxLayout* layout, const std::string& name, int lower, int upper, int min, int max);

    ConnectorIn* input_img_;
    ConnectorIn* input_mask_;

    ConnectorOut* output_mask_;

    struct State : public csapex::Memento {
        int channels;
        Encoding encoding;

        cv::Scalar min;
        cv::Scalar max;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // FILTER_TOOL_DETECTION_H
