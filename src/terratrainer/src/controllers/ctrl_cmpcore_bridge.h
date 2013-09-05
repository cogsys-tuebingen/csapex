#ifndef CTRL_CMPCORE_BRIDGE_H
#define CTRL_CMPCORE_BRIDGE_H

class QImage;

/// COMPONENT
#include <computation/cmp_core.h>
#include <computation/params.hpp>
#include <graphics/qinteractive_rect.h>
#include "controller.hpp"
#include "ctrl_state_publisher.h"
#include "ctrl_state_display.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <QObject>
#include <QColor>
#include <QThread>

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
    void writeCore(YAML::Emitter &emitter) const;

    /// GETTING IMAGES
    boost::shared_ptr<QImage> rawImage();

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

public Q_SLOTS:
    void computeROIS();
    void computeGRID();
    void computeQUAD();
    void loadIMAGE  (const QString path);

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
    void spawnBar(QString title);
    void despawnBar();
    void classesCleared();

private:
    /// THREADING
    QThread                *thread_;
    mutable boost::mutex    cc_mutex_;
    CMPCore::Ptr            cc_;
    QStatePublisher::Ptr    publisher_;
    QStateDisplay::Ptr      state_display_;

    enum Dirty{CLEAN, DIRTY, QUAD_DIRTY, GRID_DIRTY};
    Dirty dirty_;

    bool                      recalc_quad_;
    bool                      recalc_grid_;

    /// STORAGE
    std::map<int, QColor>     classes_;

    /// CLASS MANAGEMENT
    void removeClassIndex    (const int id);

};

#endif // CTRL_CMPCORE_BRIDGE_H
