#ifndef IMAGE_PADDING_H
#define IMAGE_PADDING_H

/// PROJECT
#include <csapex_vision/filter.h>

/// SYSTEM
#include <QSlider>

namespace csapex
{

class ImagePadding : public BoxedObject
{
    Q_OBJECT

public:
    ImagePadding();

    void fill(QBoxLayout *layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    void messageArrived(ConnectorIn* source);
    void update();

private:
    QSlider* slider;

    ConnectorIn* input_;
    ConnectorOut* output_;
    ConnectorOut* output_mask_;

    struct State : public Memento {
        int border;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;

};

} /// NAMESPACE

#endif // IMAGE_PADDING_H
