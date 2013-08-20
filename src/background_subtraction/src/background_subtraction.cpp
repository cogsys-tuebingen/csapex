/// HEADER
#include "background_subtraction.h"

/// COMPONENT
#include "adaptive_background_remover.h"
#include "opencv_background_remover.h"
#include "simple_background_remover.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in.h>
#include <csapex/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::BackgroundSubtraction, csapex::BoxedObject)

using namespace csapex;
using namespace background_subtraction;

BackgroundSubtraction::BackgroundSubtraction()
    : reset_btn_(NULL)
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("BackgroundSubtraction"));
}

void BackgroundSubtraction::fill(QBoxLayout *layout)
{
    if(reset_btn_ == NULL) {
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        out_masked_ = new ConnectorOut(box_, 0);
        out_masked_->setLabel("Masked Image");
        box_->addOutput(out_masked_);
        out_mask_ = new ConnectorOut(box_, 1);
        out_mask_->setLabel("Mask");
        box_->addOutput(out_mask_);
        out_bg_ = new ConnectorOut(box_, 2);
        out_bg_->setLabel("Background");
        box_->addOutput(out_bg_);


        available_removers_ = new QComboBox;
        available_removers_->addItem("OpenCV");
        available_removers_->addItem("Simple");
        available_removers_->addItem("Adaptive");

        layout->addWidget(available_removers_);
        connect(available_removers_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateRemover()));

        s_threshold_ = QtHelper::makeSlider(layout, "Threshold", 20, 0, 100);
        s_open_ = QtHelper::makeSlider(layout, "Open", 0, 0, 20);
        s_close_ = QtHelper::makeSlider(layout, "Close", 0, 0, 20);

        {
            QVBoxLayout* optional_layout = new QVBoxLayout;
            optional_["Adaptive"] = new QWidget;
            optional_["Adaptive"]->setLayout(optional_layout);
            optional_["Adaptive"]->setContentsMargins(0, 0, 0, 0);
            optional_layout->setContentsMargins(0, 0, 0, 0);

            s_max_dist_ = QtHelper::makeDoubleSlider(optional_layout, "Max. Distance", 4.0, 0.0, 50.0, 0.1);
            s_max_std_dev_ = QtHelper::makeDoubleSlider(optional_layout, "Max. Std. Dev.", 4.0, 0.0, 50.0, 0.1);
            s_decay_ = QtHelper::makeDoubleSlider(optional_layout, "Decay", 0.7, 0.0, 1.0, 0.01);
            layout->addWidget(optional_["Adaptive"]);

            connect(s_max_dist_, SIGNAL(valueChanged(double)), this, SLOT(updateSettings()));
            connect(s_max_std_dev_, SIGNAL(valueChanged(double)), this, SLOT(updateSettings()));
            connect(s_decay_, SIGNAL(valueChanged(double)), this, SLOT(updateSettings()));
        }

        {
            QVBoxLayout* optional_layout = new QVBoxLayout;
            optional_["OpenCV"] = new QWidget;
            optional_["OpenCV"]->setLayout(optional_layout);
            optional_["OpenCV"]->setContentsMargins(0, 0, 0, 0);
            optional_layout->setContentsMargins(0, 0, 0, 0);

            s_rate_ = QtHelper::makeDoubleSlider(optional_layout, "Rate", 0.002, 0.0, 0.1, 0.001);
            s_history_ = QtHelper::makeSlider(optional_layout, "history", 16, 0, 256);
            layout->addWidget(optional_["OpenCV"]);

            connect(s_rate_, SIGNAL(valueChanged(double)), this, SLOT(updateSettings()));
            connect(s_history_, SIGNAL(valueChanged(int)), this, SLOT(updateSettings()));
        }

        connect(s_threshold_, SIGNAL(valueChanged(int)), this, SLOT(updateSettings()));
        connect(s_open_, SIGNAL(valueChanged(int)), this, SLOT(updateSettings()));
        connect(s_close_, SIGNAL(valueChanged(int)), this, SLOT(updateSettings()));


        reset_btn_ = new QPushButton("Set Background");
        layout->addWidget(reset_btn_);
        connect(reset_btn_, SIGNAL(clicked()), this, SLOT(setBackground()));

        updateRemover();
    }
}

void BackgroundSubtraction::updateRemover()
{
    std::string type = available_removers_->currentText().toStdString();
    std::cout << "remover = " << type << std::endl;

    state.type = type;

    QMutexLocker lock(&remover_mutex);
    if(type == "OpenCV") {
        remover.reset(new OpenCvBackgroundRemover);
    } else if(type == "Simple") {
        remover.reset(new SimpleBackgroundRemover);
    } else if(type == "Adaptive") {
        remover.reset(new AdaptiveBackgroundRemover);
    } else {
        setError(true, std::string("unknown background remover type: ") + type);
        return;
    }

    hideOtherThan(type);

    setBackground();
}

void BackgroundSubtraction::hideOtherThan(const std::string &id)
{
    typedef std::pair<std::string, QWidget*> PAIR;
    foreach(PAIR p, optional_) {
        if(p.first == id) {
            p.second->show();
        } else {
            p.second->hide();
        }
    }
}

void BackgroundSubtraction::updateSettings()
{
    if(!signalsBlocked()) {
        state.cfg.threshold = s_threshold_->value();
        state.cfg.open = s_open_->value();
        state.cfg.close = s_close_->value();
        state.cfg.max_dist = s_max_dist_->doubleValue();
        state.cfg.max_std_dev = s_max_std_dev_->doubleValue();
        state.cfg.decay = s_decay_->doubleValue();

        state.rate = s_rate_->doubleValue();
        state.history = s_history_->value();

        remover->applyConfig(state.cfg);

        boost::shared_ptr<OpenCvBackgroundRemover> opencv = boost::dynamic_pointer_cast<OpenCvBackgroundRemover> (remover);
        if(opencv) {
            opencv->setRate(state.rate);
            opencv->setHistory(state.history);
        }
    }
}

void BackgroundSubtraction::messageArrived(ConnectorIn *source)
{
    connection_types::CvMatMessage::Ptr frame = boost::dynamic_pointer_cast<connection_types::CvMatMessage> (source->getMessage());
    assert(frame);

    connection_types::CvMatMessage::Ptr filtered(new connection_types::CvMatMessage);
    connection_types::CvMatMessage::Ptr mask(new connection_types::CvMatMessage);
    {
        QMutexLocker lock(&remover_mutex);
        if(!bg_frame) {
            bg_frame.reset(new connection_types::CvMatMessage);
            frame->value.copyTo(bg_frame->value);

            remover->setBackground(bg_frame->value);
        }

        remover->filter(frame->value, filtered->value, mask->value);

        if(out_bg_->isConnected()) {
            connection_types::CvMatMessage::Ptr bg(new connection_types::CvMatMessage);
            remover->getBackground().copyTo(bg->value);
            out_bg_->publish(bg);
        }
    }

    out_masked_->publish(filtered);
    out_mask_->publish(mask);
}

void BackgroundSubtraction::setBackground()
{
    if(bg_frame) {
        bg_frame.reset((connection_types::CvMatMessage*) NULL);
    }
}


void BackgroundSubtraction::State::writeYaml(YAML::Emitter& out) const
{
    out << YAML::Key << "type" << YAML::Value << type;

    out << YAML::Key << "max_dist" << YAML::Value << cfg.max_dist;
    out << YAML::Key << "max_std_dev" << YAML::Value << cfg.max_std_dev;
    out << YAML::Key << "decay" << YAML::Value << cfg.decay;
    out << YAML::Key << "threshold" << YAML::Value << cfg.threshold;
    out << YAML::Key << "close" << YAML::Value << cfg.close;
    out << YAML::Key << "open" << YAML::Value << cfg.open;

    out << YAML::Key << "rate" << YAML::Value << rate;
    out << YAML::Key << "history" << YAML::Value << history;

}

void BackgroundSubtraction::State::readYaml(const YAML::Node& node)
{
    if(node.FindValue("type"))
        node["type"] >> type;

    if(node.FindValue("max_dist"))
        node["max_dist"] >> cfg.max_dist;
    if(node.FindValue("max_std_dev"))
        node["max_std_dev"] >> cfg.max_std_dev;
    if(node.FindValue("decay"))
        node["decay"] >> cfg.decay;
    if(node.FindValue("threshold"))
        node["threshold"] >> cfg.threshold;
    if(node.FindValue("close"))
        node["close"] >> cfg.close;
    if(node.FindValue("open"))
        node["open"] >> cfg.open;

    if(node.FindValue("rate"))
        node["rate"] >> rate;
    if(node.FindValue("history"))
        node["history"] >> history;
}

Memento::Ptr BackgroundSubtraction::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state;

    return memento;
}

void BackgroundSubtraction::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    for(unsigned idx = 0, n = available_removers_->children().size(); idx <= n; ++idx) {
        if(available_removers_->itemText(idx).toStdString() == state.type) {
            available_removers_->setCurrentIndex(idx);
            break;
        }
    }


    blockSignals(true);

    s_max_dist_->setDoubleValue(state.cfg.max_dist);
    s_max_std_dev_->setDoubleValue(state.cfg.max_std_dev);
    s_decay_->setDoubleValue(state.cfg.decay);
    s_threshold_->setValue(state.cfg.threshold);
    s_close_->setValue(state.cfg.close);
    s_open_->setValue(state.cfg.open);

    s_rate_->setDoubleValue(state.rate);
    s_history_->setValue(state.history);

    blockSignals(false);
}
