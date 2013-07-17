#ifndef FILTER_PERSPECTIVE_H
#define FILTER_PERSPECTIVE_H

/// COMPONENT
#include <vision_evaluator/filter.h>
#include <utils/LibCvTools/perspective_transform.h>

/// SYSTEM
class QDoubleSlider;

namespace vision_plugins {

class PerspectiveTransform : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    PerspectiveTransform();

    virtual void filter(cv::Mat &img, cv::Mat &mask);
    virtual void insert(QBoxLayout *parent);

    void setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

public Q_SLOTS:
    void update();


private:
    PerspectiveTransformer transformer_;

    QDoubleSlider *slider_rot_x_;
    QDoubleSlider *slider_rot_y_;
    QDoubleSlider *slider_rot_z_;
    QDoubleSlider *slider_distance_;
    QDoubleSlider *slider_focal_length_;

    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Emitter &out) const;
    public:
        double rot_x;
        double rot_y;
        double rot_z;
        double foca;
        double dist;
    };

    State state_;

};

}
#endif // FILTER_PERSPECTIVE_H
