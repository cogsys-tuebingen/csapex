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

    /// LOAD CLASSIFIER AND CLASSES
    void   load (const std::map<int, int> &classes, const std::map<int, QString> &infos,
                 const std::vector<QColor> &colors,
                 const std::string forestPath);

    /// CLASS UPDATES AND COLOR MANAGEMENT
    void    updateClass      (const int oldID, const int newID);
    void    removeClass      (const int id);
    void    addClass         (const int classID, const int colorID);
    void    addInfo          (const int classID, const QString &class_info);
    void    updateColor      (const int classID, const int colorID);
    void    updateInfo       (const int classID, const QString &class_info);
    int     getColorID       (const int classID);
    QString getInfo          (const int classID);
    void    extendPallete    (const QColor &color);

    QColor getColor          (const int pal_index);
    QColor getColorByClass   (const int class_ID);

    std::vector<int> getClassIDs();
    int              getClassCount();

    /// RETURN THE STATE
    void getClassIndex  (std::map<int, int> &map);
    void getClassInfos  (std::map<int, QString> &map);
    void getColorPalette(std::vector<QColor> &palette);


    /// PARAMETERS
    void setExtractorParams (CMPExtractorParams &params);
    void setClassifierParams(const CMPForestParams &params);
    void setGridParams      (const CMPGridParams &params);
    void setQuadParams      (const CMPQuadParams &params);

    /// COMPUTATION
    void compute            (const std::vector<cv_roi::TerraROI> &rois);
    void computeGrid        ();
    void computeQuadtree    ();

    void getGrid(std::vector<cv_roi::TerraROI> &cells);
    void getQuadtree(std::vector<cv_roi::TerraROI> &regions);

Q_SIGNALS:
    void imageLoaded        ();
    void classifierReloaded ();
    void classAdded         (int newID);
    void classUpdate        (int oldID, int newID);
    void classRemoved       (int classID);
    void colorUpdate        (int classID);
    void trainingFinished   ();
    void feedbackFinished   ();

public Q_SLOTS:
    void loadImage          (const QString path);
    void loadClassifier     (const QString path);
    void saveClassifier     (const QString path);
    void saveClassifierRaw  (const QString path);

private:
   CMPCore::Ptr              cc_;

   /// STORAGE
   std::map<int, int>        classes_;
   std::map<int, QString>    class_infos_;
   std::vector<QColor>       classes_colors_;

   /// CLASS MANAGEMENT
   void removeClassIndex    (const int id);
   void addClassIndex       (const int id, const int colorId);
   void removeClassInfo     (const int id);
   void addClassInfo        (const int id, const QString &info);

};

#endif // CTRL_CMPCORE_BRIDGE_H
