#include "ctrl_cmpcore_bridge.h"
#include <iostream>
#include <fstream>
#include <common/QtCvImageConverter.h>
#include <QFileInfo>
#include <QDir>
#include <computation/yaml.hpp>

typedef QtCvImageConverter::Converter<QImage, boost::shared_ptr> QImageConverter;

CMPCoreBridge::CMPCoreBridge(CMPCore::Ptr ptr) :
    dirty_(CLEAN),
    cc_(ptr)
{
}

boost::shared_ptr<QImage> CMPCoreBridge::rawImage()
{
    return QImageConverter::mat2QImage(cc_->getImage());
}

void CMPCoreBridge::load(const std::map<int,int>& classes, const std::map<int, QString> &infos,
                         const std::vector<QColor> &colors, const std::string forestPath)
{
    classes_        = classes;
    class_infos_    = infos;
    classes_colors_ = colors;
    cc_->load(forestPath,getClassIDs());
}

void CMPCoreBridge::updateClass(const int oldID, const int newID)
{
    if(classes_.find(oldID) == classes_.end())
        return;
    if(class_infos_.find(oldID) != class_infos_.end()) {
        QString info = class_infos_[oldID];
        removeClassInfo(oldID);
        addClassInfo(newID, info);
    }

    int colorID = classes_[oldID];
    removeClassIndex(oldID);
    addClassIndex(newID, colorID);
    Q_EMIT classUpdate(oldID, newID);
}

void CMPCoreBridge::removeClass(int id)
{
    removeClassIndex(id);
    Q_EMIT classRemoved(id);
}

void CMPCoreBridge::addClass(const int classID, const int colorID)
{
    addClassIndex(classID, colorID);
    Q_EMIT classAdded(classID);
}

void CMPCoreBridge::addInfo(const int classID, const QString &class_info)
{
    addClassInfo(classID, class_info);
}

void CMPCoreBridge::updateColor(const int classID, const int colorID)
{
    if(classes_.find(classID) != classes_.end())
        classes_[classID] = colorID;
    Q_EMIT colorUpdate(classID);
}

void CMPCoreBridge::updateInfo(const int classID, const QString &class_info)
{
    if(class_infos_.find(classID) != class_infos_.end())
        class_infos_[classID] = class_info;
}

int CMPCoreBridge::getColorID(const int classID)
{
    return classes_[classID];
}

QString CMPCoreBridge::getInfo(const int classID)
{
    if(class_infos_.find(classID) != class_infos_.end())
        return class_infos_[classID];
    return "";
}

void CMPCoreBridge::extendPallete(const QColor &color)
{
    classes_colors_.push_back(color);
}

QColor CMPCoreBridge::getColor(const int pal_index)
{
    return classes_colors_[pal_index];
}

QColor CMPCoreBridge::getColorByClass(const int class_ID)
{
    if(classes_.find(class_ID) == classes_.end())
        return QColor();

    return classes_colors_[classes_[class_ID]];
}

std::vector<int> CMPCoreBridge::getClassIDs()
{
    std::vector<int> ids;
    for(std::map<int, int>::iterator it = classes_.begin() ; it != classes_.end() ; it++)
        ids.push_back(it->first);
    return ids;
}

int CMPCoreBridge::getClassCount()
{
    return classes_.size();
}

void CMPCoreBridge::getClassIndex(std::map<int,int>  &map)
{
    map = classes_;
}

void CMPCoreBridge::getClassInfos(std::map<int, QString> &map)
{
    map = class_infos_;
}

void CMPCoreBridge::getColorPalette(std::vector<QColor> &palette)
{
    palette = classes_colors_;
}

void CMPCoreBridge::loadImage(const QString path)
{
    if(cc_->loadImage(path.toUtf8().data()))
        Q_EMIT imageLoaded();
}

void CMPCoreBridge::loadClassifier(const QString path)
{
    std::ifstream in(path.toUtf8().constData());
    CMPYAML::readFile(in, cc_.get(), this);
    in.close();
    cc_->computeQuadtree();
    Q_EMIT classifierReloaded();
}

void CMPCoreBridge::saveClassifier(const QString path)
{
    std::ofstream out(path.toUtf8().constData());
    if(!out.is_open()) {
        std::cerr << "Couldn't write file!" << std::endl;
    }

    CMPYAML::writeFile(out, cc_.get(), this);
    out.close();
}

void CMPCoreBridge::saveClassifierRaw(const QString path)
{
    QFile file(cc_->forestPath().c_str());
    if(file.exists()) {
        if(!file.copy(path)) {
            std::cerr << "File couldn't be saved! Check your rights!" << std::endl;
        }
    } else {
        std::cerr << "No forest computed yet!" << std::endl;
    }
}

void CMPCoreBridge::setExtractorParams(CMPExtractorParams &params)
{
    cc_->setExtractorParameters(params);
}

void CMPCoreBridge::setForestParams(const CMPForestParams &params)
{
    dirty_ = QUAD_DIRTY;
    cc_->setRandomForestParams(params);
}

void CMPCoreBridge::setGridParams(const CMPGridParams &params)
{
    dirty_ = GRID_DIRTY;
    cc_->setGridParameters(params);
}

void CMPCoreBridge::setQuadParams(const CMPQuadParams &params)
{
    cc_->setQuadParameters(params);
}

void CMPCoreBridge::setKeyPointParams(const CMPKeypointParams &params)
{
    cc_->setKeyPointParameters(params);
}

void CMPCoreBridge::setROIs(const std::vector<cv_roi::TerraROI> &rois)
{
    cc_->setRois(rois);
}

void CMPCoreBridge::saveROIs(QString path)
{
    /// SAVE
    for(std::map<int,int>::iterator it = classes_.begin() ; it != classes_.end() ; it++) {
        int id = it->first;
        QString id_path = path + "/" + QString::number(id);
        QDir directory(id_path);
        if(!directory.exists() && !QDir().mkdir(id_path)) {
            std::cerr << "Folder cannot be created '" << id_path.toUtf8().constData() << "' !" << std::endl;
            return;
        }
    }
    cc_->saveRois(std::string(path.toUtf8().constData()) + "/");
}


void CMPCoreBridge::compute()
{
    cc_->compute();
    dirty_ = DIRTY;
    Q_EMIT computeFinished();
}

void CMPCoreBridge::computeGrid()
{
    if(dirty_ == DIRTY) {
        cc_->computeGrid();
        dirty_ = QUAD_DIRTY;
    } else if(dirty_ == GRID_DIRTY) {
        cc_->computeGrid();
        dirty_ = CLEAN;
    }
}

void CMPCoreBridge::computeQuadtree()
{
    if(dirty_ == DIRTY) {
        cc_->computeQuadtree();
        dirty_ = GRID_DIRTY;
    } else if(dirty_ == QUAD_DIRTY) {
        cc_->computeQuadtree();
        dirty_ = CLEAN;
    }
}

void CMPCoreBridge::getGrid(std::vector<cv_roi::TerraROI> &cells)
{
    cc_->getGrid(cells);
}

void CMPCoreBridge::getQuadtree(std::vector<cv_roi::TerraROI> &regions)
{
    cc_->getQuad(regions);
}

void CMPCoreBridge::removeClassIndex(const int id)
{
    std::map<int, int>::iterator entry = classes_.find(id);
    classes_.erase(entry);
    cc_->removeClass(id);
}

void CMPCoreBridge::addClassIndex(const int id, const int colorId)
{
    std::pair<int, int> entry(id, colorId);
    classes_.insert(entry);
    cc_->addClass(id);
}

void CMPCoreBridge::removeClassInfo(const int id)
{
    std::map<int, QString>::iterator entry = class_infos_.find(id);
    class_infos_.erase(entry);
    cc_->removeClass(id);
}

void CMPCoreBridge::addClassInfo(const int id, const QString &info)
{
    std::pair<int, QString> entry(id, info);
    class_infos_.insert(entry);
}

