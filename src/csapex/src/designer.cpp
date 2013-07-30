/// HEADER
#include <csapex/designer.h>

/// PROJECT
#include "ui_designer.h"
#include <csapex/connector.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/selector_proxy.h>
#include <csapex/box_manager.h>
#include <csapex/box.h>
#include <csapex/qt_helper.hpp>
#include <csapex/designerio.h>
#include <csapex/command_meta.h>
#include <csapex/command_delete_box.h>
#include <csapex/stream_interceptor.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QResizeEvent>
#include <QMenu>
#include <QScrollBar>
#include <QFileDialog>

using namespace csapex;

Q_DECLARE_METATYPE(QSharedPointer<QImage>)

Designer::Designer(QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer)
{
    StreamInterceptor::instance().start();

    qRegisterMetaType<QSharedPointer<QImage> >("QSharedPointer<QImage>");

    StreamInterceptor::instance();

    ui->setupUi(this);

    setCurrentConfig(DesignerIO::default_config);

    BoxManager::instance().reload();

    QObject::connect(&BoxManager::instance(), SIGNAL(stateChanged()), this, SIGNAL(stateChanged()));
}

Designer::~Designer()
{
    StreamInterceptor::instance().stop();
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

    QList<csapex::Box*> boxes = findChildren<csapex::Box*> ();
    BOOST_FOREACH(csapex::Box* box, boxes) {
        Command::Ptr cmd(new command::DeleteBox(box));
        clear->add(cmd);
    }
    BoxManager::instance().execute(clear);
}
