#include "randomc.h"
#include "imagefeatureextraction.h"
#include <cv.h>
#include "ipoint.h"
#include "LTP.h"
#include "surflib.h"
#include <iostream>
#include <QDir>
#include <QTextStream>
using namespace std;

ImageFeatureExtraction::ImageFeatureExtraction()
{
}

bool ImageFeatureExtraction::extract(QString colorsFile, QString dirOriginal, QString dirGroundTruth, Descriptor descriptor, int size, double parameter, bool usePosition)
{
    // Load color table ///////////////////////////////////////////////////////////////////////////

    if (!loadColorTable(colorsFile))
    {
        cerr << "Could not load color table " << colorsFile.toStdString() << endl;
        return false;
    }

    cout << "Color table " << colorsFile.toStdString() << " loaded\n";
    printLabels();

    // Load images ////////////////////////////////////////////////////////////////////////////////

    getImages(dirOriginal, mOriginalImages, mOriginalFileInfos);

    if (mOriginalImages.size() > 0)
    {
        cout << dirOriginal.toStdString() << ": " << mOriginalImages.size() << endl;
    }
    else
    {
        cerr << "No images available at " << dirOriginal.toStdString() << endl;
    }

    getImages(dirGroundTruth, mGroundTruthImages, mGroundTruthFileInfos);

    if (mGroundTruthImages.size() > 0)
    {
        cout << dirGroundTruth.toStdString() << ": " << mGroundTruthImages.size() << endl;
    }
    else
    {
        cerr << "No images available at " << dirGroundTruth.toStdString() << endl;
    }

    // Set descriptor /////////////////////////////////////////////////////////////////////////////

    int descSize = 0;
    QString name;
    QString type;

    switch (descriptor)
    {
        case DESCRIPTOR_LTP:
            descSize = 512;
            name = "LTP";
            type = "integer";
            break;

        case DESCRIPTOR_TSURF:
            descSize = 64;
            name = "TSURF";
            type = "real";
            break;

        case DESCRIPTOR_XSURF:
            descSize = 64;
            name = "XSURF";
            type = "real";
            break;

        default:
            cerr << "Unknown descriptor with id " << descriptor << endl;
            return 1;
    }
    // Open output file ///////////////////////////////////////////////////////////////////////////

    QString relation = "terrain_" + name + (usePosition ? "+" : "") + "_" + QString::number(size) + "_" + QString::number(parameter);
    QString arffFilename = QString(dirGroundTruth) + "/" + relation + ".arff";
    QFile arffFile(arffFilename);

    if (!arffFile.open(QFile::WriteOnly | QFile::Text))
    {
        cerr << "Could not open output file " << arffFilename.toStdString() << endl;
        return 1;
    }

    cout << "Output: " << arffFilename.toStdString() << endl;

    // Write header ///////////////////////////////////////////////////////////////////////////////

    QTextStream fileWriter;
    fileWriter.setDevice(&arffFile);
    fileWriter << "@relation " + relation + "\n\n";

    if (usePosition)
    {
        fileWriter << "@attribute pos_x integer\n"
                   << "@attribute pos_y integer\n";
    }

    for (int d = 0; d != descSize; ++d)
    {
        fileWriter << "@attribute " << name << "_" << QString::number(d) << " " << type << endl;
    }

    fileWriter << "@attribute class {";

    for (int i = 0; i != mLabels.size(); ++i)
    {
        fileWriter << mLabels[i] << (i != mLabels.size() - 1 ? ", " : "");
    }

    fileWriter << "}" << endl << endl;
    fileWriter << "@data" << endl;

    // Feature extraction /////////////////////////////////////////////////////////////////////////

    QVector<QVector<QString> > features(mLabels.size());
    TRandomMersenne random(time(0)); // uniform random number generator
    random.RandomInit(time(0));

    int w = mGroundTruthImages[0].size().width();
    int h = mGroundTruthImages[0].size().height();

    if (descriptor == DESCRIPTOR_LTP || descriptor == DESCRIPTOR_TSURF)
    {
        int padX = (w % size) / 2;
        int padY = (h % size) / 2;
        int n2 = w / size;
        int n1 = h / size;

        // Generate keypoints //////////////////////////////////////////////////////////////////////

        vector<Ipoint> ipt0(n1 * n2);
        int k = 0;

        for (int i = 0; i != n1; ++i)
        {
            for (int j = 0; j != n2; ++j)
            {
                ipt0[k].x = padX + (j + 0.5) * size;
                ipt0[k].y = padY + (i + 0.5) * size;
                ipt0[k].scale = parameter;
                ++k;
            }
        }

        // Extract features ///////////////////////////////////////////////////////////////////////

        // for all images

        for (int img = 0; img != mGroundTruthImages.size(); ++img)
        {
            QImage original = mOriginalImages[img];
            QImage groundTruth = mGroundTruthImages[img];

            // Get ground truth ///////////////////////////////////////////////////////////////////

            vector<int> classes(n1 * n2);
            float size2 = 0.5f * size * size;
            int counter = 0;

            // for all cells (u, v)

            for (int v = 0; v < n1; ++v)
            {
                for (int u = 0; u < n2; ++u)
                {
                    vector<int> counts(mLabels.size(), 0);

                    // for all pixels (x, y) in (u, v)

                    int begY = padY + v * size;

                    for (int y = begY; y != begY + size; ++y)
                    {
                        int begX = padX + u * size;

                        for (int x = begX; x != begX + size; ++x)
                        {
                            int l = color2label(groundTruth.pixel(x, y));

                            if (l != -1)
                            {
                                ++counts[l];
                            }
                        }
                    }

                    // Label of c_uv

                    int label = -1;

                    for (int c = 0; c != mLabels.size(); ++c)
                    {
                        if (counts[c] > size2)
                        {
                            label = c;
                        }
                    }

                    classes[counter++] = label;

                    if (descriptor == DESCRIPTOR_LTP && label >= 0)
                    {
                        // Integer patch

                        int x0 = padX + u * size;
                        int y0 = padY + v * size;
                        int x1 = x0 + size;
                        int y1 = y0 + size;

                        int *patchGray = new int[size * size];
                        k = 0;

                        for (int y = y0, y_ = 0; y != y1; ++y, ++y_)
                        {
                            for (int x = x0, x_ = 0; x != x1; ++x, ++x_)
                            {
                                patchGray[k++] = qGray(original.pixel(x, y));
                            }
                        }

                        LTP histo;
                        ltp_histogram(patchGray, size, size, parameter, histo, false);
                        delete[] patchGray;

                        // Save feature data

                        QString ftrs;
                        ftrs = QString::number(u) + "," + QString::number(v) + ",";

                        for (int d = 0; d != descSize; ++d)
                        {
                            ftrs += QString::number(histo[d]) + ",";
                        }

                        ftrs += mLabels[label];
                        features[label].push_back(ftrs);
                    }
                }
            }

            if (descriptor == DESCRIPTOR_TSURF)
            {
                // IplImage

                IplImage *p = cvCreateImage(cvSize(original.width(), original.height()), IPL_DEPTH_8U, 1);
                k = 0;

                for (int y = 0; y != original.height(); ++y)
                {
                    for (int x = 0; x != original.width(); ++x)
                    {
                        p->imageData[k++] = qGray(original.pixel(x, y));
                    }
                }

                vector<Ipoint> ipt(ipt0);
                surfDes(p, ipt, true);
                cvReleaseImage(&p);

                // Save feature data

                for (unsigned kp = 0; kp != ipt.size(); ++kp)
                {
                    if (classes[kp] >= 0)
                    {
                        QString ftrs;

                        for (int d = 0; d != descSize; ++d)
                        {
                            ftrs += QString::number(ipt[kp][d]) + ",";
                        }

                        ftrs += mLabels[classes[kp]];
                        features[classes[kp]].push_back(ftrs);
                    }
                }
            }
        }
    }
    else if (descriptor == DESCRIPTOR_XSURF)
    {
        // for all images

        for (int img = 0; img != mGroundTruthImages.size(); ++img)
        {
            QImage original = mOriginalImages[img];
            QImage groundTruth = mGroundTruthImages[img];
            vector<Ipoint> ipt;
            vector<int> labelsGT;

            // Sampling ///////////////////////////////////////////////////////////////////////////

            Ipoint ip;
            ip.scale = parameter;
            int counter = 0;

            while (ipt.size() < size_t(size) && counter < 2 * size)
            {
                int x = random.Random() * w;
                int y = random.Random() * h;
                int l = color2label(groundTruth.pixel(x, y));

                if (l != -1)
                {
                    labelsGT.push_back(l);

                    ip.x = x;
                    ip.y = y;
                    ipt.push_back(ip);
                }

                ++counter;
            }

            // Create IplImage /////////////////////////////////////////////////////////////////////

            IplImage *p = cvCreateImage(cvSize(original.width(), original.height()), IPL_DEPTH_8U, 1);
            int k = 0;

            for (int y = 0; y != original.height(); ++y)
            {
                for (int x = 0; x != original.width(); ++x)
                {
                    p->imageData[k++] = qGray(original.pixel(x, y));
                }
            }

            // Extract features ////////////////////////////////////////////////////////////////////

            surfDes(p, ipt, true);
            cvReleaseImage(&p);

            // Save feature data //////////////////////////////////////////////////////////////////

            for (unsigned kp = 0; kp != ipt.size(); ++kp)
            {
                QString ftrs;

                if (usePosition)
                {
                    ftrs = QString::number(ipt[kp].x) + "," + QString::number(ipt[kp].y) + ",";
                }

                for (int d = 0; d != descSize; ++d)
                {
                    ftrs += QString::number(ipt[kp][d]) + ",";
                }

                ftrs += mLabels[labelsGT[kp]];
                features[labelsGT[kp]].push_back(ftrs);
            }
        }
    }

    // Shuffle ////////////////////////////////////////////////////////////////////////////////////

    srand(unsigned(time(NULL)));

    for (int i = 0; i != features.size(); ++i)
    {
        random_shuffle(features[i].begin(), features[i].end());
    }

    // Min size

    vector<int> sizes;

    for (int i = 0; i != features.size(); ++i)
    {
        sizes.push_back(features[i].size());
    }

    int minSize = *min_element(sizes.begin(), sizes.end());

    if (features.size() > 0)
    {
        cout << "Instances: ";

        for (int fi = 0; fi != features.size(); ++fi)
        {
            cout << features[fi].size() << (features[fi].size() == minSize ? "*" : "") << (fi != features.size() - 1 ? ", " : "\n");
        }
    }

    // Concatenate

    QVector<QString> all;

    for (int i = 0; i != features.size(); ++i)
    {
        for (int j = 0; j != minSize; ++j)
        {
            all.push_back(features[i][j]);
        }
    }

    // Shuffle
    random_shuffle(all.begin(), all.end());

    // Write data

    for (int i = 0; i != all.size(); ++i)
    {
        fileWriter << all[i] << endl;
    }

    cout << "Total: " << all.size() << endl << endl;

    return true;
}

int ImageFeatureExtraction::color2label(QColor col)
{
    int label = -1;

    for (int i = 0; i != mLabelColors.size(); ++i)
    {
        if (col == mLabelColors[i])
        {
            label = i;
            break;
        }
    }

    return label;
}

// Get the images and file infos contained in the given directory
void ImageFeatureExtraction::getImages(QString directoryName, QVector<QImage> &images, QVector<QFileInfo> &fileInfos)
{
    images.clear();
    fileInfos.clear();

    QDir dir(directoryName);
    QFileInfoList fileInfoList = dir.entryInfoList(QDir::Files);

    for (int i = 0; i != fileInfoList.size(); ++i)
    {
        QFileInfo fileInfo = fileInfoList.at(i);
        QImage image(fileInfo.absoluteFilePath());

        if (!image.isNull())
        {
            images.push_back(image);
            fileInfos.push_back(fileInfo);
        }
    }
}

bool ImageFeatureExtraction::loadColorTable(QString filename)
{
    QFile file(filename);

    if (!file.open(QFile::ReadOnly | QFile::Text))
    {
        return false;
    }

    QTextStream in(&file);
    QString line = in.readLine();
    mLabels.clear();
    mLabelColors.clear();

    while (!line.isEmpty())
    {
        QStringList data = line.split(QRegExp("\\s+"));

        if (data.size() == 4)
        {
            mLabels.push_back(data[0]);
            mLabelColors.push_back(QColor(data[1].toInt(), data[2].toInt(), data[3].toInt()));
        }

        line = in.readLine();
    }

    return mLabels.size();
}

void ImageFeatureExtraction::printLabels()
{
    cout << "Labels: ";

    for (int i = 0; i != mLabels.size(); ++i)
    {
        cout << mLabels[i].toStdString() << (i < mLabels.size() - 1 ? ", " : "\n");
    }
}
