#ifndef CSAPEX_PLUGIN_H
#define CSAPEX_PLUGIN_H

/// COMPONENT
#include "background_remover.h"
#include <background_subtraction/GlobalConfig.h>

/// PROJECT
#include <csapex/boxed_object.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/qdouble_slider.h>

/// SYSTEM
#include <QPushButton>
#include <QComboBox>
#include <QMutex>
#include <QSlider>

namespace csapex
{

class ConnectorIn;
class ConnectorOut;

class BackgroundSubtraction : public BoxedObject
{
    Q_OBJECT

public:
    BackgroundSubtraction();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private Q_SLOTS:
    void updateRemover();
    void updateSettings();
    void setBackground();

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);

private:
    void hideOtherThan(const std::string& id);

private:
    struct State : public Memento {        
        std::string type;
        background_subtraction::GlobalConfig cfg;
        double rate;
        int history;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;

    boost::shared_ptr<background_subtraction::BackgroundRemover> remover;
    QMutex remover_mutex;

    connection_types::CvMatMessage::Ptr bg_frame;

    QPushButton* reset_btn_;
    QComboBox* available_removers_;

    QSlider* s_threshold_;
    QSlider* s_open_;
    QSlider* s_close_;

    std::map<std::string, QWidget*> optional_;

    QDoubleSlider* s_max_dist_;
    QDoubleSlider* s_max_std_dev_;
    QDoubleSlider* s_decay_;

    QDoubleSlider* s_rate_;
    QSlider* s_history_;


    ConnectorIn* input_;
    ConnectorOut* out_masked_;
    ConnectorOut* out_mask_;
    ConnectorOut* out_bg_;
};

}

#endif // CSAPEX_PLUGIN_H
