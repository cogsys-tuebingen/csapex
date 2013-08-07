#include "ctrl_cmpcore_bridge.h"
#include <iostream>
#include <fstream>
#include <common/QtCvImageConverter.h>
#include <QFileInfo>
#include <QDir>
#include <computation/yaml.hpp>

typedef QtCvImageConverter::Converter<QImage, boost::shared_ptr> QImageConverter;

CMPCoreBridge::CMPCoreBridge(CMPCore::Ptr ptr) :
    cc_(ptr)
{
}

boost::shared_ptr<QImage> CMPCoreBridge::rawImage()
{
    return QImageConverter::mat2QImage(cc_->getImage());
}

void CMPCoreBridge::load(const std::map<int,int>& classes, const std::vector<QColor> &colors, const std::string forestPath)
{
    classes_ = classes;
    cc_->load(forestPath,getClassIDs());
    classes_colors_ = colors;
}

void CMPCoreBridge::updateClass(const int oldID, const int newID)
{
    if(classes_.find(oldID) == classes_.end())
        return;
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

void CMPCoreBridge::updateColor(const int classID, const int colorID)
{
    if(classes_.find(classID) != classes_.end())
        classes_[classID] = colorID;
    Q_EMIT colorUpdate(classID);
}

int CMPCoreBridge::getColorRef(const int classID)
{
    return classes_[classID];
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
    switch(params.type) {
    case CMPExtractorParams::ORB:
        cc_->setExtractorParameters(static_cast<CMPParamsORB&>(params));
        break;
    case CMPExtractorParams::SURF:
        cc_->setExtractorParameters(static_cast<CMPParamsSURF&>(params));
        break;
    case CMPExtractorParams::SIFT:
        cc_->setExtractorParameters(static_cast<CMPParamsSIFT&>(params));
        break;
    case CMPExtractorParams::BRIEF:
        cc_->setExtractorParameters(static_cast<CMPParamsBRIEF&>(params));
        break;
    case CMPExtractorParams::BRISK:
        cc_->setExtractorParameters(static_cast<CMPParamsBRISK&>(params));
        break;
    case CMPExtractorParams::FREAK:
        cc_->setExtractorParameters(static_cast<CMPParamsFREAK&>(params));
        break;
    }
}

void CMPCoreBridge::setClassifierParams(CMPForestParams &params)
{
    cc_->setRandomForestParams(params);
}

void CMPCoreBridge::compute(const std::vector<CMPCore::ROI> &rois)
{
    cc_->setRois(rois);
    cc_->compute();
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
