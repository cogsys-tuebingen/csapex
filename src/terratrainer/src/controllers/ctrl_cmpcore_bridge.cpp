#include "ctrl_cmpcore_bridge.h"
#include <iostream>
#include <fstream>
#include <ios>
#include <common/QtCvImageConverter.h>
#include <QFileInfo>
#include <QDir>
#include <computation/yaml.hpp>

typedef QtCvImageConverter::Converter<QImage, boost::shared_ptr> QImageConverter;

CMPCoreBridge::CMPCoreBridge(CMPCore::Ptr ptr) :
    recalc_quad_(true),
    recalc_grid_(true),
    cc_(ptr)
{
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

        cc_->reload();

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
    cc_->write(emitter);
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
    cc_->removeClass(oldID);
    cc_->addClass(newID);

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
    cc_->addClass(classID);

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

void CMPCoreBridge::loadImage(const QString path)
{
    recalc_grid_ = true;
    recalc_quad_ = true;
    if(cc_->loadImage(path.toUtf8().data()))
        Q_EMIT imageLoaded();
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

void CMPCoreBridge::compute()
{
    cc_->compute();

    recalc_quad_ = true;
    recalc_grid_ = true;

    Q_EMIT computeFinished();
}

void CMPCoreBridge::computeGrid()
{
    if(recalc_grid_) {
        if(cc_->hasComputedModel()) {
            cc_->computeGrid();
            recalc_grid_ = false;
        } else {
            std::cerr << "No valid classifier model available!" << std::endl;
        }
    }

    Q_EMIT computeGridFinished();
}

void CMPCoreBridge::computeQuadtree()
{
    if(recalc_quad_) {
        if(cc_->hasComputedModel()) {
            cc_->computeQuadtree();
            recalc_quad_ = false;
        } else {
            std::cerr << "No valid classifier model available!" << std::endl;
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
    cc_->getGrid(cells);
}

void CMPCoreBridge::getQuadtree(std::vector<cv_roi::TerraROI> &regions)
{
    cc_->getQuad(regions);
}

void CMPCoreBridge::removeClassIndex(const int id)
{
    std::map<int, QColor>::iterator entry = classes_.find(id);
    classes_.erase(entry);
    cc_->removeClass(id);
}
