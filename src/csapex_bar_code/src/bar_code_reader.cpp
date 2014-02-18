/// HEADER
#include "bar_code_reader.h"

/// COMPONENT
#include <csapex_core_plugins/string_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <zbar.h>

PLUGINLIB_EXPORT_CLASS(csapex::BarCodeReader, csapex::Node)

using namespace csapex;
using namespace connection_types;

BarCodeReader::BarCodeReader()
    : lost(false), forget(0)
{
    Tag::createIfNotExists("Bar Code");
    addTag(Tag::get("Bar Code"));

    addParameter(param::ParameterFactory::declareBool("republish", false));
}

void BarCodeReader::process()
{
    CvMatMessage::Ptr msg = in_img->getMessage<CvMatMessage>();

    if(msg->value.channels() != 1) {
        throw std::runtime_error("Input must be 1-channel image!");
    }

    bool republish = param<bool>("republish");

    zbar::ImageScanner scanner;
    // configure the reader
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // wrap image data
    int w = msg->value.cols;
    int h = msg->value.rows;
    zbar::Image image(w, h, "Y800", msg->value.data, w * h);

    // scan the image for barcodes
    int n = scanner.scan(image);
    if(n == 0) {
        if(!data_.empty() && !lost) {
            lost = true;
            forget = 30;
        }
    }

    if(lost) {
        if(forget > 0) {
            --forget;
        }
        if(forget == 0) {
            data_ = "";
            lost = false;
        }
    }

    VectorMessage::Ptr out(VectorMessage::make<RoiMessage>());

    // extract results
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) {

        const zbar::Symbol& sym = *symbol;
        std::string data = sym.get_data();

        {
            int x = std::numeric_limits<int>::max();
            int y = std::numeric_limits<int>::max();
            int X = std::numeric_limits<int>::min();
            int Y = std::numeric_limits<int>::min();

            for(int i = 0; i < sym.get_location_size(); ++i) {
                int px = sym.get_location_x(i);
                int py = sym.get_location_y(i);
                x = std::min(x, px);
                X = std::max(X, px);
                y = std::min(y, py);
                Y = std::max(Y, py);
            }

            RoiMessage::Ptr msg(new RoiMessage);
            cv::Rect rect(x,y, X-x, Y-y);
            if(rect.x >= 0 && rect.y >= 0) {
                msg->value = Roi(rect, cv::Scalar(0,0,0));
                out->value.push_back(msg);
            }
        }

        if(data == data_) {
            if(lost) {
                lost = false;
            }

            if(!republish) {
                continue;
            }
        }

        StringMessage::Ptr msg(new StringMessage);
        msg->value = data;
        out_str->publish(msg);

        data_ = data;
    }

    if(!out->value.empty()) {
        out_roi->publish(out);
    }

    // clean up
    image.set_data(NULL, 0);

}


void BarCodeReader::setup()
{
    setSynchronizedInputs(true);

    in_img = addInput<CvMatMessage>("Image", false, true);

    out_str = addOutput<StringMessage>("String");
    out_roi = addOutput<VectorMessage, RoiMessage>("ROIs");
}
