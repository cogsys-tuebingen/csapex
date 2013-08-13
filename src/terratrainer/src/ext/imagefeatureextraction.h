#ifndef IMAGE_FEATURE_EXTRACTION
#define IMAGE_FEATURE_EXTRACTION

#include <QColor>
#include <QFileInfo>
#include <QImage>
#include <QString>
#include <QVector>

/// The ImageFeatureExtraction class provides functions for extracting features from images.
class ImageFeatureExtraction
{
public:
    ImageFeatureExtraction();

    enum Descriptor {DESCRIPTOR_LTP, DESCRIPTOR_TSURF, DESCRIPTOR_XSURF};
    bool extract(QString colorsFile, QString dirOriginal, QString dirGroundTruth, Descriptor desc, int size, double parameter, bool usePosition = false);

private:
    int color2label(QColor col);
    void getImages(QString directoryName, QVector<QImage> &images, QVector<QFileInfo> &fileInfos);
    bool loadColorTable(QString filename);
    void printLabels();

    QVector<QFileInfo> mGroundTruthFileInfos;
    QVector<QImage> mGroundTruthImages;
    QVector<QColor> mLabelColors;
    QVector<QString> mLabels;
    QVector<QFileInfo> mOriginalFileInfos;
    QVector<QImage> mOriginalImages;
};

#endif
