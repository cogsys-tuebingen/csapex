#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <vision_evaluator/filter.h>

/// SYSTEM
#include <QSlider>

namespace vision_evaluator
{

class FilterBlur : public Filter
{
    Q_OBJECT

public:
    FilterBlur();

    virtual void filter(cv::Mat& img, cv::Mat& mask);
    virtual void insert(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private Q_SLOTS:
    void update(int slot);

private:
    QSlider* slider;

    struct State : public Memento {
        int blur;
        virtual void writeYaml(YAML::Emitter& out) const {
            out << YAML::Key << "blur" << YAML::Value << blur;
        }
        virtual void readYaml(const YAML::Node& node) {
            node["blur"] >> blur;
            std::cout << "read blur: " << blur << std::endl;
        }
    };

    State state;

};

} /// NAMESPACE

#endif // FilterBlur_H
