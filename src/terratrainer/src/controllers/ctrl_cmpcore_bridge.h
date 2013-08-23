#ifndef CTRL_CMPCORE_BRIDGE_H
#define CTRL_CMPCORE_BRIDGE_H
/// COMPONENT
#include <computation/cmp_core.h>
#include <computation/params.hpp>
#include <graphics/qinteractive_rect.h>
#include "controller.hpp"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <QImage>
#include <QColor>

class CMPCoreBridge : public QObject, public Controller
{
    Q_OBJECT
public:
    typedef boost::shared_ptr<CMPCoreBridge> Ptr;
    struct QROI {
        QRectF      bounding;
        int         classId;
        double      rotation;
    };

    CMPCoreBridge(CMPCore::Ptr ptr);

    void read(const YAML::Node &document);
    void write(YAML::Emitter &emitter) const;


    /// GETTING IMAGES
    boost::shared_ptr<QImage> rawImage();

    /// LOAD CLASSIFIER AND CLASSES
    void   load (const std::map<int, int> &classes, const std::map<int, QString> &infos,
                 const std::vector<QColor> &colors,
                 const std::string forestPath);

    /// CLASS UPDATES AND COLOR MANAGEMENT
    void    classUpdate      (const int oldID, const int newID);
    void    classRemove      (const int id);
    void    classAdd         (const int classID, const QColor &color);
    void    classClear       ();
    void    colorUpdate      (const int classID, const QColor &color);
    void    colorClear       ();
    QColor  colorGet         (const int class_ID);

    std::vector<int>        getClassIDs();
    int                     getClassCount();

    /// PARAMETERS
    void setExtractorParams (cv_extraction::ExtractorParams &params);
    void setForestParams    (const CMPForestParams &params);
    void setGridParams      (const CMPGridParams &params);
    void setQuadParams      (const CMPQuadParams &params);
    void setKeyPointParams  (const cv_extraction::KeypointParams &params);

    /// COMPUTATION
    void setROIs            (const std::vector<cv_roi::TerraROI> &rois);

    bool recalcGrid();
    bool recalcQuad();
    void getGrid(std::vector<cv_roi::TerraROI> &cells);
    void getQuadtree(std::vector<cv_roi::TerraROI> &regions);
    void compute            ();
    void computeGrid        ();
    void computeQuadtree    ();

    void loadImage          (const QString path);

Q_SIGNALS:
    void imageLoaded        ();
    void classifierReloaded ();
    void classAdded         (int newID);
    void classUpdated       (int oldID, int newID);
    void classRemoved       (int classID);
    void colorUpdate        (int classID);
    void computeFinished();
    void computeGridFinished();
    void computeQuadFinished();


private:
    enum Dirty{CLEAN, DIRTY, QUAD_DIRTY, GRID_DIRTY};
    Dirty dirty_;

    bool                      recalc_quad_;
    bool                      recalc_grid_;

    CMPCore::Ptr              cc_;

    /// STORAGE
    std::map<int, QColor>     classes_;

    /// CLASS MANAGEMENT
    void removeClassIndex    (const int id);
    //void addClassIndex       (const int id, const int colorId);

};

#endif // CTRL_CMPCORE_BRIDGE_H
