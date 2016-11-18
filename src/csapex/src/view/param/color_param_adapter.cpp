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
#include <QApplication>
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

}

ColorParameterAdapter::ColorParameterAdapter(param::ColorParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), color_p_(p)
{

}

QWidget* ColorParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QPushButton> btn = new QPushButton;

    btn->setStyleSheet(toColorSS(color_p_->value()));

    btn->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(btn.data(), &QPushButton::customContextMenuRequested,
                     [=](const QPoint& point){ customContextMenuRequested(btn, point); });


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

        QColorDialog diag(QApplication::activeWindow());
        diag.setCurrentColor(init);
        diag.setModal(true);

//        diag.setOptions(QColorDialog::DontUseNativeDialog | QColorDialog::ShowAlphaChannel);
//        diag.setVisible(true);
//        diag.setOptions(QColorDialog::DontUseNativeDialog);

//        diag.setAttribute(Qt::WA_WState_Visible, true);
//        diag.setAttribute(Qt::WA_WState_Hidden, false);

        //        diag.setAttribute(Qt::WA_WState_ExplicitShowHide);

        if(!diag.exec()) {
            return;
        }

        QColor color = diag.selectedColor();
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
    connectInGuiThread(p_->parameter_changed, [this, btn](param::Parameter*){
        if(!color_p_ || !btn) {
            return;
        }
        btn->setStyleSheet(toColorSS(color_p_->value()));
    });

    return btn;
}



void ColorParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
    context_handler->addAction(new QAction("reset to default", context_handler), [this](){
        color_p_->set(color_p_->def());
    });
}

