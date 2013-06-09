/// HEADER
#include "designer.h"

/// PROJECT
#include "ui_designer.h"
#include "connector.h"
#include "selector_proxy.h"
#include "box_manager.h"

/// SYSTEM
#include <QResizeEvent>
#include <QMenu>
#include <iostream>

using namespace vision_evaluator;

Designer::Designer(QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer)
{
    ui->setupUi(this);

    BoxManager::instance().fill(ui->widget_selection->layout());
}
