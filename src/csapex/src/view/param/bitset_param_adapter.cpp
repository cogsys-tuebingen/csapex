/// HEADER
#include <csapex/view/param/bitset_param_adapter.h>

/// PROJECT
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>
#include <csapex/view/utility/qt_helper.hpp>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QPushButton>
#include <iostream>

using namespace csapex;

BitSetParameterAdapter::BitSetParameterAdapter(param::BitSetParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), bitset_p_(p)
{

}

QWidget* BitSetParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    group = new QGroupBox(display_name.c_str());
    QVBoxLayout* l = new QVBoxLayout;
    group->setLayout(l);

    setupAgain();

    layout->addWidget(group);

    connectInGuiThread(bitset_p_->scope_changed, [this](param::Parameter*) {
        if(!bitset_p_ || !group) {
            return;
        }

        disconnect();
        QtHelper::clearLayout(group->layout());
        setupAgain();
    });
    return group;
}

void BitSetParameterAdapter::setupAgain()
{
    for(int i = 0; i < bitset_p_->noParameters(); ++i) {
        std::string str = bitset_p_->getName(i);
        QCheckBox* item = new QCheckBox(QString::fromStdString(str));
        group->layout()->addWidget(item);
        if(bitset_p_->isSet(str)) {
            item->setChecked(true);
        }

        // ui change -> model
        QObject::connect(item, &QCheckBox::toggled, [this, item, str](bool checked) {
            if(!bitset_p_ || !item) {
                return;
            }
            auto v = std::make_pair(str, checked);
            command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), v);
            executeCommand(update_parameter);
        });

        // model change -> ui
        connectInGuiThread(bitset_p_->parameter_changed, [this, item, str](param::Parameter*) {
            if(!bitset_p_ || !item) {
                return;
            }
            item->blockSignals(true);
            item->setChecked(bitset_p_->isSet(str));
            item->blockSignals(false);
        });
    }

}


void BitSetParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
    context_handler->addAction(new QAction("reset to default", context_handler), [this](){
        bitset_p_->set(bitset_p_->def());
    });
}
