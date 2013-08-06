#ifndef CTRL_CMPCORE_BRIDGE_H
#define CTRL_CMPCORE_BRIDGE_H
/// COMPONENT
#include <computation/cmp_core.h>
#include <computation/params.hpp>
#include <graphics/qinteractive_rect.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <QImage>
#include <QColor>



class CMPCoreBridge : public QObject
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

    /// GETTING IMAGES
    boost::shared_ptr<QImage> rawImage();

    /// CLASS SETTINGS
    void   removeClass      (const int id);
    void   addClass         (const int classID, const int colorID);
    void   updateClassColor (const int classID, const int colorID);
    int    getColorRef      (const int classID);
    void   extendPallete    (const QColor &color);

    QColor getColor       (const int pal_index);
    QColor getColorByClass(const int class_ID);

    std::vector<int> getClassIDs();
    int              getClassCount();

    /// GENERAL SETTINGS
    void setExtractorParams(CMPExtractorParams &params);
    void compute(const std::vector<CMPCore::ROI> &rois);

Q_SIGNALS:
    void imageLoaded();
    void classifierReloaded();
    void classUpdate();
    void computationFinished();

public Q_SLOTS:
    void loadImage(const QString path);
    void loadClass(const QString path);
    bool saveClass(const QString path, const QString filename);
private:
   CMPCore::Ptr cc_;

   bool updateMetaForestPath(const std::string &meta_file, const std::string &forest_name);

   std::map<int, int>      classes_;
   std::vector<QColor>     classes_colors_;
};

#endif // CTRL_CMPCORE_BRIDGE_H
