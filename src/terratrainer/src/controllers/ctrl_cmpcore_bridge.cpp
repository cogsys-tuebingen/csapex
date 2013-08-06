#include "ctrl_cmpcore_bridge.h"
#include <iostream>
#include <fstream>
#include <common/QtCvImageConverter.h>
#include <QFileInfo>
#include <QDir>

typedef QtCvImageConverter::Converter<QImage, boost::shared_ptr> QImageConverter;

CMPCoreBridge::CMPCoreBridge(CMPCore::Ptr ptr) :
    cc_(ptr)
{
}

boost::shared_ptr<QImage> CMPCoreBridge::rawImage()
{
    return QImageConverter::mat2QImage(cc_->getImage());
}

void CMPCoreBridge::removeClass(int id)
{
    std::map<int, int>::iterator entry = classes_.find(id);
    classes_.erase(entry);
    cc_->removeClass(id);
    Q_EMIT classUpdate();
    saveClass("/tmp/", "test");
}

void CMPCoreBridge::addClass(const int classID, const int colorID)
{
    std::pair<int, int> entry(classID, colorID);
    classes_.insert(entry);
    cc_->addClass(classID);
    Q_EMIT classUpdate();
}

void CMPCoreBridge::updateClassColor(const int classID, const int colorID)
{
    if(classes_.find(classID) != classes_.end())
        classes_[classID] = colorID;
    Q_EMIT classUpdate();
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

void CMPCoreBridge::loadImage(const QString path)
{
    if(cc_->loadImage(path.toUtf8().data()))
        Q_EMIT imageLoaded();
}

void CMPCoreBridge::loadClass(const QString path)
{
    QFileInfo info(path);
    QString dir  = info.dir().absolutePath();
    QString meta = info.fileName();
    cc_->loadForest(dir.toUtf8().constData(), meta.toUtf8().constData());
}

bool CMPCoreBridge::saveClass(const QString path, const QString filename)
{
    /// "/" !!!
    bool copied = true;
    QFile meta(cc_->metaPath().c_str());
    QFile forest(cc_->forestPath().c_str());

    std::string path_       = path.toUtf8().constData();
    std::string filename_   = filename.toUtf8().constData();
    std::string meta_path   =  path_ + filename_ + ".meta.yaml";
    std::string forest_name = filename_ + ".forest.yaml";
    copied &= meta.copy(meta_path.c_str());
    updateMetaForestPath(meta_path, forest_name);
    copied &= forest.copy((path_ + forest_name).c_str());
    return copied;
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

void CMPCoreBridge::compute(const std::vector<CMPCore::ROI> &rois)
{
    cc_->setRois(rois);
    cc_->compute();
}

bool CMPCoreBridge::updateMetaForestPath(const std::string &meta_file, const std::string &forest_name)
{
    std::ifstream in(meta_file.c_str());
    if(!in.is_open()) {
        std::cerr << "Error opening file to update! READING" << std::endl;
        return false;
    }

    try {
        YAML::Parser  parser(in);
        YAML::Node    doc;
        parser.GetNextDocument(doc);

        int classCount;
        doc["classCount"] >> classCount;

        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "classCount" << classCount;
        emitter << YAML::Key << "forest" << YAML::Value << forest_name;
        emitter << YAML::Key << "classes";
        emitter << YAML::BeginSeq;
        const YAML::Node &node = doc["classes"];
        for(YAML::Iterator it = node.begin() ; it != node.end(); it++)  {
            int classID;
            *it >> classID;
            emitter << classID;
        }
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;
        in.close();

        std::ofstream of(meta_file.c_str());
        if(!of.is_open()) {
            std::cerr << "Error opening file to update! WRITING" << std::endl;
            return false;
        }
        of << emitter.c_str();
        of.close();
    } catch (YAML::Exception e) {
        std::cerr << "Error update file! " << e.what() << std::endl;
        return false;
    }
    return true;
}
