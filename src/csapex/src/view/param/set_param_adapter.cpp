/// HEADER
#include <csapex/view/param/set_param_adapter.h>

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
#include <QComboBox>
#include <iostream>

using namespace csapex;




SetParameterAdapter::SetParameterAdapter(param::SetParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), set_p_(p)
{

}

QWidget* SetParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QPointer<QComboBox> combo = new QComboBox;

    updateSetParameterScope(combo);
    layout->addLayout(QtHelper::wrap(display_name, combo, context_handler));

    // ui change -> model
    QObject::connect(combo.data(), static_cast<void(QComboBox::*)(const QString&)>(&QComboBox::currentIndexChanged),
                     [this](const QString& val) {
        if(!p_) {
            return;
        }

        if(!val.isEmpty()) {
//            p->setByName(combo->currentText().toStdString());
            command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>
                    (AUUID(p_->getUUID()), std::make_pair(val.toStdString(), true));
            executeCommand(update_parameter);
        }
    });

    // model change -> ui
    connectInGuiThread(set_p_->parameter_changed, [this, combo](param::Parameter*) {
        if(!set_p_ || !combo) {
            return;
        }
        int index = combo->findText(QString::fromStdString(set_p_->getText()));
        if(index >= 0) {
            combo->blockSignals(true);
            combo->setCurrentIndex(index);
            combo->blockSignals(false);
        }
    });

    connectInGuiThread(set_p_->scope_changed, [this, combo](param::Parameter*) {
        updateSetParameterScope(combo);
    });

    return combo;
}


void SetParameterAdapter::updateSetParameterScope(QPointer<QComboBox> combo)
{
    if(!set_p_ || !combo) {
        return;
    }

    int current = 0;
    combo->clear();
    std::string selected;
    try {
        selected = set_p_->getText();
    } catch(const std::exception& e) {
        selected = "";
    }

    combo->blockSignals(true);
    for(int i = 0; i < set_p_->noParameters(); ++i) {
        std::string str = set_p_->getText(i);
        combo->addItem(QString::fromStdString(str));

        if(str == selected) {
            current = i;
        }
    }
    combo->setCurrentIndex(current);
    combo->blockSignals(false);
    combo->update();
}


void SetParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
    context_handler->addAction(new QAction("reset to default", context_handler), [this](){
        set_p_->setByName(set_p_->defText());
    });
}
