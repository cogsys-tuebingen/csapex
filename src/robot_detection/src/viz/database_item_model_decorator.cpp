/// HEADER
#include "database_item_model_decorator.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <db/database_io.h>
#include <utils/LibUtil/QtCvImageConverter.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <QFileDialog>
#include <QMessageBox>
#include <QTreeWidgetItem>
#include <QTimer>



const std::string DatabaseItemModelDecorator::FILE_EXTENSION("Database (*.db)");


DatabaseItemModelDecorator::DatabaseItemModelDecorator(Database* db, QTreeWidget* list)
    : db(db), tree(list), current_target(NULL), image_size(128, 128)
{
    delete_shortcut = new QShortcut(list);
    delete_shortcut->setKey(QKeySequence("Del"));

    db->changed.connect(boost::bind(&DatabaseItemModelDecorator::rebuildRequest, this));
    db->replaced.connect(boost::bind(&DatabaseItemModelDecorator::rebuildRequest, this));

    repaint_timer = new QTimer(this);
    repaint_timer->setInterval(1000);
    repaint_timer->setSingleShot(true);
    repaint_timer->stop();

    connect(repaint_timer, SIGNAL(timeout()), this, SLOT(rebuild()), Qt::QueuedConnection);

//    QObject::connect(this, SIGNAL(rebuildInGuiThread()), this, SLOT(rebuild()), Qt::QueuedConnection);
//    QObject::connect(this, SIGNAL(appendNewItemsInGuiThread()), this, SLOT(append()), Qt::QueuedConnection);
//    QObject::connect(this, SIGNAL(changeItemInGuiThread(int)), this, SLOT(change(int)), Qt::QueuedConnection);

    QObject::connect(delete_shortcut, SIGNAL(activated()), this, SLOT(deleteCurrentRow()));

    current_db_file.setFileName(db->getConfig().db_file.c_str());

    if(current_db_file.exists()) {
        loadCurrentFile();
    }
}

DatabaseItemModelDecorator::~DatabaseItemModelDecorator()
{
    delete delete_shortcut;
}

void DatabaseItemModelDecorator::rebuildRequest()
{
    if(!repaint_timer->isActive()) {
        repaint_timer->start();
    }
}

void DatabaseItemModelDecorator::rebuild()
{
    repaint_timer->stop();

    cleanItems();

    boost::function<void(int,const std::string&)> cf = boost::bind(&DatabaseItemModelDecorator::addComposite, this, _1, _2);
    boost::function<void(int,MatchablePose*)> lf = boost::bind(&DatabaseItemModelDecorator::addLeaf, this, _1, _2);

    current_target = NULL;
    tree->setColumnCount(3);
    tree->setColumnWidth(0, image_size.width() * 2);
    tree->setIconSize(image_size);
    db->traversePoses(cf, lf);

    tree->expandAll();
    tree->repaint();
}

void DatabaseItemModelDecorator::addComposite(int level, const std::string& label)
{
    QTreeWidgetItem* next = makeItem(level-1, label);
    if(current_target == NULL || level == 0) {
        tree->addTopLevelItem(next);
        current_target = next;
    } else {
    }
}

void DatabaseItemModelDecorator::addLeaf(int level, MatchablePose* leaf)
{

    QTreeWidgetItem* item = makeItem(level-1, leaf);
    if(current_target == NULL) {
        tree->addTopLevelItem(item);
    } else {
        current_target->addChild(item);
    }
}

QTreeWidgetItem* DatabaseItemModelDecorator::makeItem(int level, MatchablePose* p)
{
    QSharedPointer<QImage> img;

    if(p == MatchablePose::NULL_POSE || p->image.empty()) {
        img = QSharedPointer<QImage>(new QImage(32, 32, QImage::Format_ARGB32));
        img->fill(0xFFFF0000);
    } else {
        IplImage ipl_img = p->image;
        img = QtCvImageConverter::Converter<QImage, QSharedPointer>::ipl2QImage(&ipl_img);
    }

    QTreeWidgetItem* item = new QTreeWidgetItem;

    item->setSizeHint(level, image_size);
    item->setIcon(level, QIcon(QPixmap::fromImage(*img)));
    if(p->saved) {
        item->setBackgroundColor(level, QColor::fromRgb(255, 255, 255));
    } else {
        item->setBackgroundColor(level, QColor::fromRgb(255, 100, 100));
    }

    std::stringstream ss;
    ss << round(p->orientation.toDegrees());
    item->setText(level+1, ss.str().c_str());

    return item;
}

QTreeWidgetItem* DatabaseItemModelDecorator::makeItem(int level, const std::string& text)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(tree);
    item->setText(level+1, text.c_str());

    return item;
}

bool DatabaseItemModelDecorator::deleteCurrentRow()
{
//    int i = tree->currentIndex().row();
//    if(i >= 0 && i < db->count()) {
//        db->getPose(i);
//        return true;
//    }

    return false;
}


void DatabaseItemModelDecorator::cleanItems()
{
    tree->clear();
}


void DatabaseItemModelDecorator::save()
{
    if(!current_db_file.open(QIODevice::WriteOnly)) {
        ERROR("Unable to open file " << current_db_file.fileName().toStdString());
    } else {
        INFO("save to " << current_db_file.fileName().toStdString());
        Config c = db->getConfig();
        c.db_file = current_db_file.fileName().toStdString();
        db->applyConfig(c);
        DatabaseIO::save(c.db_file, db);

        current_db_file.close();
    }
}

void DatabaseItemModelDecorator::saveAs()
{
    QString f = QFileDialog::getSaveFileName(tree->parentWidget(), tr("Save Database"), "", tr(FILE_EXTENSION.c_str()));
    if(!f.isEmpty()) {
        current_db_file.setFileName(f);
        save();
        current_db_file.close();
    }
}

void DatabaseItemModelDecorator::loadCurrentFile()
{
    std::string file = current_db_file.fileName().toStdString();

    Config cfg = db->getConfig();
    if(cfg.db_file != file) {
        INFO("load " << file);
        cfg.db_file = file;

        if(db != NULL) {
            delete db;
        }

        DatabaseIO::load(cfg.db_file, db);
    }
    current_db_file.close();
}

void DatabaseItemModelDecorator::load()
{
    QString f = QFileDialog::getOpenFileName(tree->parentWidget(), tr("Open Database"), "", tr(FILE_EXTENSION.c_str()));
    if(!f.isEmpty()) {
        current_db_file.setFileName(f);

        if(!current_db_file.open(QIODevice::ReadOnly)) {
            ERROR("Unable to open file " << current_db_file.fileName().toStdString());
        } else {
            loadCurrentFile();
        }
    }
}
