/// HEADER
#include "bar_code_reader.h"

/// COMPONENT
#include <csapex_core_plugins/string_message.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <zbar.h>

PLUGINLIB_EXPORT_CLASS(csapex::BarCodeReader, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

BarCodeReader::BarCodeReader()
    : lost(false), forget(0)
{
    Tag::createIfNotExists("Bar Code");
    addTag(Tag::get("Bar Code"));
}

void BarCodeReader::allConnectorsArrived()
{
    CvMatMessage::Ptr msg = in_img->getMessage<CvMatMessage>();

    if(msg->value.channels() != 1) {
        throw std::runtime_error("Input must be 1-channel image!");
    }

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

    // extract results
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) {

        std::string data = symbol->get_data();

        if(data == data_) {
            if(lost) {
                lost = false;
            }
            continue;
        }

        StringMessage::Ptr msg(new StringMessage);
        msg->value = data;
        out_str->publish(msg);

        data_ = data;
        break;
    }

    // clean up
    image.set_data(NULL, 0);
}


void BarCodeReader::fill(QBoxLayout* layout)
{
    setSynchronizedInputs(true);

    in_img = addInput<CvMatMessage>("Image");

    out_str = addOutput<StringMessage>("String");
}
