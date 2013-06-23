#ifndef IMAGE_PROVIDER_SET_H
#define IMAGE_PROVIDER_SET_H

/// COMPONENT
#include "image_provider.h"

/// SYSTEM
#include <QPushButton>
#include <QSlider>

namespace vision_evaluator
{

class ImageProviderSet : public ImageProvider
{
    Q_OBJECT

protected:
    ImageProviderSet();
    virtual ~ImageProviderSet();

public:
    virtual void insert(QBoxLayout* layout);
    virtual void update_gui(QFrame* additional_holder);
    virtual void next(cv::Mat& img, cv::Mat& mask);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);


protected Q_SLOTS:
    void showFrame();
    void setPlaying(bool playing);

protected: // abstract
    virtual void reallyNext(cv::Mat& img, cv::Mat& mask) = 0;

protected:

    struct State : public Memento {
        bool playing_;
        int current_frame;

        virtual void writeYaml(YAML::Emitter& out) const {
            out << YAML::Key << "playing" << YAML::Value << playing_;
            out << YAML::Key << "current_frame" << YAML::Value << current_frame;
        }
        virtual void readYaml(const YAML::Node& node) {
            node["playing"] >> playing_;
            node["current_frame"] >> current_frame;
        }
    };

    State state;

    cv::Mat last_frame_;

    QPushButton* play_pause_;
    QSlider* slider_;


    double fps_;
    double frames_;
    int next_frame;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_SET_H
