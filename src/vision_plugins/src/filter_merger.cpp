#include "filter_merger.h"

/// PROJECT
#include <QLabel>
#include <designer/box.h>
#include <vision_evaluator/box_manager.h>
#include <vision_evaluator/command_add_connection.h>
#include <vision_evaluator/command_delete_connection.h>
#include <evaluator/messages_default.hpp>
#include <designer/connector_in.h>
#include <designer/connector_out.h>
#include <designer/command_meta.h>
#include <designer/command.h>
#include <vision_evaluator/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::Merger, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

Merger::Merger() :
    output_(NULL)
{
}

void Merger::fill(QBoxLayout *layout)
{
    if(output_ == NULL) {
        /// add output
        output_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_);

        /// inputs
        input_count_ = QtHelper::makeSpinBox(layout, "Inputs: ", 2, 2, 10);
        QSpinBox::connect(input_count_, SIGNAL(valueChanged(int)), this, SLOT(updateInputs(int)));
        updateInputs(2);
    }
}

void Merger::messageArrived(ConnectorIn *source)
{
    input_arrivals_[source] = true;

    if(!gotAllArrivals())
        return;


    std::vector<cv::Mat> msgs;
    collectMessage(msgs);
    cv::Mat out_img;
    try {
        cv::merge(msgs, out_img);
        CvMatMessage::Ptr out_msg(new CvMatMessage);
        out_msg->value = out_img;
        output_->publish(out_msg);
    } catch (cv::Exception e) {
        std::cerr << " ERROR " << e.what() << std::endl;
    }

    resetInputArrivals();



}

void Merger::updateInputs(int value)
{
    int to_erase = inputs_.size() - value;
    if(to_erase > 0) {
        for(int i = 0 ; i < to_erase ; i++) {
            ConnectorIn *ptr = inputs_.back();
            inputs_.pop_back();
            box_->removeInput(ptr);

            for(std::map<ConnectorIn*,bool>::iterator it = input_arrivals_.begin() ; it != input_arrivals_.end() ; it++) {
                if(it->first == ptr) {
                    input_arrivals_.erase(it, it);
                    break;
                }
            }
        }
    } else {
        int to_add = -to_erase;
        for(int i = 0 ; i < to_add ; i++) {
            ConnectorIn* input = new ConnectorIn(box_, inputs_.size() + i);
            box_->addInput(input);
            assert(QObject::connect(input, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*))));

            inputs_.push_back(input);
            input_arrivals_.insert(std::pair<ConnectorIn*, bool>(input, false));

        }
    }

}

void Merger::collectMessage(std::vector<cv::Mat> &messages)
{
    for(std::vector<ConnectorIn*>::iterator it = inputs_.begin() ; it != inputs_.end() ; it++) {
        if((*it)->isConnected()) {
            CvMatMessage::Ptr msg = boost::dynamic_pointer_cast<CvMatMessage> ((*it)->getMessage());
            messages.push_back(msg->value);
        }
    }
}


bool Merger::gotAllArrivals()
{
    bool ready = true;
    for(std::vector<ConnectorIn*>::iterator it = inputs_.begin() ; it != inputs_.end() ; it++) {
        ready &= input_arrivals_[*it] || !(*it)->isConnected();
    }
    return ready;
}

void Merger::resetInputArrivals()
{
    for(std::map<ConnectorIn*,bool>::iterator it = input_arrivals_.begin() ; it != input_arrivals_.end() ; it++) {
        it->second = false;
    }
}
