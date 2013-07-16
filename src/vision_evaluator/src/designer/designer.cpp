/// HEADER
#include "designer.h"

/// PROJECT
#include "ui_designer.h"
#include "connector.h"
#include "connector_in.h"
#include "connector_out.h"
#include "selector_proxy.h"
#include "box_manager.h"
#include "box.h"
#include "../qt_helper.hpp"
#include "designerio.h"
#include "command_meta.h"
#include "command_delete_box.h"
#include <utils/stream_interceptor.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QResizeEvent>
#include <QMenu>
#include <QScrollBar>
#include <QFileDialog>

using namespace vision_evaluator;

Designer::Designer(QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer)
{
    StreamInterceptor::instance();

    ui->setupUi(this);

    setCurrentConfig(DesignerIO::default_config);

    BoxManager::instance().reload();

    QObject::connect(&BoxManager::instance(), SIGNAL(stateChanged()), this, SIGNAL(stateChanged()));
}

bool Designer::eventFilter(QObject* o, QEvent* e)
{
    return true;
}

void Designer::keyPressEvent(QKeyEvent* e)
{
    ui->designer->keyPressEvent(e);
}

void Designer::keyReleaseEvent(QKeyEvent* e)
{
    ui->designer->keyReleaseEvent(e);
}

void Designer::resizeEvent(QResizeEvent* e)
{
}

bool Designer::isDirty()
{
    return BoxManager::instance().isDirty();
}


bool Designer::canUndo()
{
    return BoxManager::instance().canUndo();
}

bool Designer::canRedo()
{
    return BoxManager::instance().canRedo();
}

void Designer::save()
{
    DesignerIO::save(this, current_config_);
}

void Designer::setCurrentConfig(const std::string& filename)
{
    current_config_ = filename;

    Q_EMIT configChanged();
}

std::string Designer::getConfig()
{
    return current_config_;
}

void Designer::saveAs()
{
    QString filename = QFileDialog::getSaveFileName(0, "Save config", current_config_.c_str(), DesignerIO::config_selector.c_str());

    if(!filename.isEmpty()) {
        DesignerIO::save(this, filename.toUtf8().constData());
        setCurrentConfig(filename.toUtf8().constData());
    }
}

void Designer::load()
{
    QString filename = QFileDialog::getOpenFileName(0, "Load config", current_config_.c_str(), DesignerIO::config_selector.c_str());

    if(QFile(filename).exists()) {
        DesignerIO::load(this, filename.toUtf8().constData());
        setCurrentConfig(filename.toUtf8().constData());
    }
}

void Designer::reload()
{
    DesignerIO::load(this, current_config_);
}

void Designer::undo()
{
    BoxManager::instance().undo();
}

void Designer::redo()
{
    BoxManager::instance().redo();
}

void Designer::clear()
{
    command::Meta::Ptr clear(new command::Meta);

    QList<vision_evaluator::Box*> boxes = findChildren<vision_evaluator::Box*> ();
    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {
        Command::Ptr cmd(new command::DeleteBox(box));
        clear->add(cmd);
    }
    BoxManager::instance().execute(clear);
}
