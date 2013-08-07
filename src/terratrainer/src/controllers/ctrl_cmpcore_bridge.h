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
    void   load (const std::map<int, int> &classes,
                 const std::vector<QColor> &colors,
                 const std::string forestPath);

    void   updateClass      (const int oldID, const int newID);
    void   removeClass      (const int id);
    void   addClass         (const int classID, const int colorID);
    void   updateColor      (const int classID, const int colorID);
    int    getColorRef      (const int classID);
    void   extendPallete    (const QColor &color);

    QColor getColor       (const int pal_index);
    QColor getColorByClass(const int class_ID);

    std::vector<int> getClassIDs();
    int              getClassCount();

    /// RETURN THE STATE
    void getClassIndex(std::map<int, int> &map);
    void getColorPalette (std::vector<QColor> &palette);

    /// GENERAL SETTINGS
    void setExtractorParams(CMPExtractorParams &params);
    void setClassifierParams(CMPForestParams &params);
    void compute(const std::vector<CMPCore::ROI> &rois);

Q_SIGNALS:
    void imageLoaded();
    void classifierReloaded();
    void classAdded  (int newID);
    void classUpdate (int oldID, int newID);
    void classRemoved(int classID);
    void colorUpdate (int classID);
    void computationFinished();

public Q_SLOTS:
    void loadImage(const QString path);
    void loadClassifier(const QString path);
    void saveClassifier(const QString path);
    void saveClassifierRaw(const QString path);
private:
   CMPCore::Ptr cc_;

   std::map<int, int>      classes_;
   std::vector<QColor>     classes_colors_;

   void removeClassIndex(const int id);
   void addClassIndex(const int id, const int colorId);

};

#endif // CTRL_CMPCORE_BRIDGE_H
