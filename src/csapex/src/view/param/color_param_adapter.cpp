/// HEADER
#include <csapex/view/param/color_param_adapter.h>

/// PROJECT
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>

/// SYSTEM
#include <QPointer>
#include <QPushButton>
#include <QColorDialog>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace csapex;

namespace {

QString toColorSS(const std::vector<int>& v) {
    std::stringstream ss;
    ss << "QPushButton {";
    ss << "background-color: #" << std::hex << std::setfill('0');
    ss << std::setw(2) << v[0];
    ss << std::setw(2) << v[1];
    ss << std::setw(2) << v[2];
    ss << std::dec << ";";
    ss << "}";

    return QString::fromStdString(ss.str());
}

void ui_updateColorParameter(param::ColorParameterWeakPtr color_p, QPointer<QPushButton> btn)
{

}

void model_updateColorParameter(param::ColorParameterWeakPtr color_p)
{
}
}

ColorParameterAdapter::ColorParameterAdapter(param::ColorParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), color_p_(p)
{

}

void ColorParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QPushButton> btn = new QPushButton;

    btn->setStyleSheet(toColorSS(color_p_->value()));

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    layout->addLayout(QtHelper::wrap(display_name, sub, context_handler));

    // ui callback
    QObject::connect(btn.data(), &QPushButton::pressed, [this, btn](){
        if(!color_p_ || !btn) {
            return;
        }

        std::vector<int> c = color_p_->value();
        QColor init(c[0], c[1], c[2]);
        QColor color = QColorDialog::getColor(init);
        if (color.isValid()) {
            std::vector<int> v(3);
            v[0] = color.red();
            v[1] = color.green();
            v[2] = color.blue();

            btn->setStyleSheet(toColorSS(v));

            command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), v);
            executeCommand(update_parameter);
        }
    });

    // model change -> ui
    connectInGuiThread(p_->parameter_changed, [this, btn](){
        if(!color_p_ || !btn) {
            return;
        }
        btn->setStyleSheet(toColorSS(color_p_->value()));
    });
}

