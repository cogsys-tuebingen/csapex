/// HEADER
#include <csapex/view/param/output_text_param_adapter.h>

/// PROJECT
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QLabel>
#include <iostream>

using namespace csapex;

OutputTextParameterAdapter::OutputTextParameterAdapter(param::OutputTextParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), op_p_(p)
{

}

namespace {
static bool isFixedPitch(const QFont & font) {
    const QFontInfo fi(font);
    return fi.fixedPitch();
}

static QFont getMonospaceFont(){
    QFont font("monospace");
    if (isFixedPitch(font)) return font;
    font.setStyleHint(QFont::Monospace);
    if (isFixedPitch(font)) return font;
    font.setStyleHint(QFont::TypeWriter);
    if (isFixedPitch(font)) return font;
    font.setFamily("courier");
    if (isFixedPitch(font)) return font;
    return font;
}
}


QWidget* OutputTextParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QLabel> label = new QLabel;
    label->setFont(getMonospaceFont());
    label->setText(QString::fromStdString(op_p_->as<std::string>()));
    layout->addWidget(label);

    // model change -> ui
    connectInGuiThread(op_p_->parameter_changed, [this, label](param::Parameter*) {
        if(op_p_ && label) {
            label->setMaximumWidth(label->parentWidget()->width());
            label->setText(QString::fromStdString(op_p_->as<std::string>()));
        }
    });
    return label;
}
