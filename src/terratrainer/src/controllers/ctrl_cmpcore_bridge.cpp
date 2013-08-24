#include "ctrl_cmpcore_bridge.h"
#include <common/QtCvImageConverter.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <fstream>
#include <QImage>
#include <QCoreApplication>

typedef QtCvImageConverter::Converter<QImage, boost::shared_ptr> QImageConverter;

CMPCoreBridge::CMPCoreBridge(CMPCore::Ptr ptr) :
    cc_(ptr),
    publisher_(new QStatePublisher),
    state_display_(new QStateDisplay),
    recalc_quad_(true),
    recalc_grid_(true)
{
    thread_ = new QThread;
    moveToThread(thread_);

    state_display_->moveToThread(QCoreApplication::instance()->thread());
    publisher_->registerListener(state_display_.get());
    cc_->setStatePublisher(publisher_);

    connect(this, SIGNAL(spawnBar(QString)), state_display_.get(), SLOT(spawnBar(QString)), Qt::QueuedConnection);
    connect(this, SIGNAL(despawnBar()), state_display_.get(), SLOT(despawnBar()), Qt::QueuedConnection);
    connect(this, SIGNAL(destroyed()), thread_, SLOT(deleteLater()));
    connect(thread_, SIGNAL(finished()), SLOT(deleteLater()));

    thread_->start(QThread::HighPriority);

}

void CMPCoreBridge::read(const YAML::Node &document)
{
    try {
        const YAML::Node &data = document["CLASSIFIER"];
        std::ofstream out(cc_->forestPath().c_str());

        if(!out.is_open()) {
            std::cerr << "Couldn't open forest file!" << std::endl;
            return;
        }

        std::string buf;
        data["DATA"] >> buf;
        out << buf;
        out.close();

        boost::mutex::scoped_try_lock lock(cc_mutex_);
        if(lock) {
            cc_->reload();
            lock.unlock();
        }

    } catch (YAML::Exception e) {
        std::cerr << "ORB Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
    }

}

void CMPCoreBridge::write(YAML::Emitter &emitter) const
{
    std::ifstream in(cc_->forestPath().c_str());
    if(!in.is_open()) {
        std::cerr << "Couldn't find trained classifier!" << std::endl;
        return;
    }

    std::stringstream buf;
    buf << in.rdbuf();
    emitter << YAML::Key << "CLASSIFIER" << YAML::Value;
    emitter << YAML::BeginMap;

    boost::mutex::scoped_try_lock lock(cc_mutex_);
    if(lock) {
        cc_->write(emitter);
        lock.unlock();
    }
    emitter << YAML::Key << "DATA" << YAML::Value;
    emitter << buf.str();
    emitter << YAML::EndMap;
    in.close();
}


boost::shared_ptr<QImage> CMPCoreBridge::rawImage()
{
    return QImageConverter::mat2QImage(cc_->getImage());
}

void CMPCoreBridge::classUpdate(const int oldID, const int newID)
{
    if(classes_.find(oldID) == classes_.end())
        return;

    QColor color = classes_[oldID];
    removeClassIndex(oldID);

    std::pair<int, QColor> entry(newID, color);
    classes_.insert(entry);

    Q_EMIT classUpdated(oldID, newID);
}

void CMPCoreBridge::classRemove(int id)
{
    removeClassIndex(id);
    Q_EMIT classRemoved(id);
}

void CMPCoreBridge::classAdd(const int classID, const QColor &color)
{
    std::pair<int, QColor> entry(classID, color);
    classes_.insert(entry);

    Q_EMIT classAdded(classID);
}

void CMPCoreBridge::colorUpdate(const int classID, const QColor &color)
{
    if(classes_.find(classID) != classes_.end())
        classes_[classID] = color;

    Q_EMIT colorUpdate(classID);
}

QColor CMPCoreBridge::colorGet(const int class_ID)
{
    if(classes_.find(class_ID) == classes_.end())
        return QColor();

    return classes_[class_ID];
}

std::vector<int> CMPCoreBridge::getClassIDs()
{
    std::vector<int> ids;
    for(std::map<int, QColor>::iterator it = classes_.begin() ; it != classes_.end() ; it++) {
        ids.push_back(it->first);
    }

    return ids;
}

int CMPCoreBridge::getClassCount()
{
    return classes_.size();
}

void CMPCoreBridge::loadIMAGE(const QString path)
{
    boost::mutex::scoped_try_lock lock(cc_mutex_);
    if(lock) {
        if(cc_->loadImage(path.toUtf8().data()))
            Q_EMIT imageLoaded();
        lock.unlock();
        recalc_grid_ = true;
        recalc_quad_ = true;
    }
}

void CMPCoreBridge::setExtractorParams(cv_extraction::ExtractorParams &params)
{
    cc_->setExtractorParameters(params);
}

void CMPCoreBridge::setForestParams(const CMPForestParams &params)
{
    cc_->setRandomForestParams(params);
}

void CMPCoreBridge::setGridParams(const CMPGridParams &params)
{
    recalc_grid_ = true;
    cc_->setGridParameters(params);
}

void CMPCoreBridge::setQuadParams(const CMPQuadParams &params)
{
    recalc_quad_ = true;
    cc_->setQuadParameters(params);
}

void CMPCoreBridge::setKeyPointParams(const cv_extraction::KeypointParams &params)
{
    cc_->setKeyPointParameters(params);
}

void CMPCoreBridge::setROIs(const std::vector<cv_roi::TerraROI> &rois)
{
    cc_->setRois(rois);
}

void CMPCoreBridge::computeROIS()
{
    boost::mutex::scoped_try_lock lock(cc_mutex_);
    if(lock) {
        Q_EMIT spawnBar("Compute ROIS");
        cc_->compute();
        recalc_quad_ = true;
        recalc_grid_ = true;
        Q_EMIT despawnBar();
    }
    Q_EMIT computeFinished();
}

void CMPCoreBridge::computeGRID()
{
    if(recalc_grid_) {
        boost::mutex::scoped_try_lock lock(cc_mutex_);
        if(lock) {
            if(cc_->hasComputedModel()) {
                cc_->computeGrid();
                recalc_grid_ = false;
            } else {
                std::cerr << "No valid classifier model available!" << std::endl;
            }
            lock.unlock();
        }
    }

    Q_EMIT computeGridFinished();
}

void CMPCoreBridge::computeQUAD()
{
    if(recalc_quad_) {
        boost::mutex::scoped_try_lock lock(cc_mutex_);
        if(lock) {
            if(cc_->hasComputedModel()) {
                cc_->computeQuadtree();
                recalc_quad_ = false;
            } else {
                std::cerr << "No valid classifier model available!" << std::endl;
            }
            lock.unlock();
        }
    }

    Q_EMIT computeQuadFinished();
}

bool CMPCoreBridge::recalcGrid()
{
    return recalc_grid_;
}

bool CMPCoreBridge::recalcQuad()
{
    return recalc_quad_;
}

void CMPCoreBridge::getGrid(std::vector<cv_roi::TerraROI> &cells)
{
    boost::mutex::scoped_try_lock lock(cc_mutex_);
    if(lock) {
        cc_->getGrid(cells);
        lock.unlock();
    }
}

void CMPCoreBridge::getQuadtree(std::vector<cv_roi::TerraROI> &regions)
{
    boost::mutex::scoped_try_lock lock(cc_mutex_);
    if(lock) {
        cc_->getQuad(regions);
        lock.unlock();
    }
}

void CMPCoreBridge::removeClassIndex(const int id)
{
    std::map<int, QColor>::iterator entry = classes_.find(id);
    classes_.erase(entry);
}
