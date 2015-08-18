#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace QtCvImageConverter {

/**
 * @brief The QTRGBConverter struct is the default rgb mapper
 */
struct QTRGBConverter {
static inline unsigned int rgb(int r, int g, int b) // set RGB value
{ return (0xffu << 24) | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff); }

static inline unsigned int rgba(int r, int g, int b, int a) // set RGBA value
{ return ((a & 0xff) << 24) | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff); }
};

/// FORWARD DECLARATION
class QImage;

static inline int getRed(unsigned rgb)
{ return ((rgb >> 16) & 0xff); }

static inline int getGreen(unsigned rgb)
{ return ((rgb >> 8) & 0xff); }

static inline int getBlue(unsigned rgb)
{ return (rgb & 0xff); }

static inline int getAlpha(unsigned rgb)
{ return rgb >> 24; }

/**
 * @brief The Converter class is a helper class template for converting QImages to cv images
 * @note It is a template so that Qt doesn't have to be linked to this library
 */
template <class TargetClass, class RGBConverter = QTRGBConverter>
class Converter
{
private:
    /**
     * @brief Tools
     */
    Converter();

public:
    /**
     * @brief mat2QImage converts an OpenCV image to a Qt image
     * @param mat OpenCV image
     * @return Qt image
     */
    static cv::Mat QImage2Mat(const TargetClass &img) {
        int h = img.height();
        int w = img.width();
        cv::Mat mat(h, w, CV_8UC3);

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                unsigned source = img.pixel(x,y);
                cv::Vec3b& target = mat.at<cv::Vec3b>(y,x);
                target[0] = getBlue(source);
                target[1] = getGreen(source);
                target[2] = getRed(source);
            }
        }
        return mat;
    }

    /**
     * @brief mat2QImage converts an OpenCV image to a Qt image
     * @param mat OpenCV image
     * @return Qt image
     */
    static TargetClass mat2QImage(const cv::Mat &mat) {
        const IplImage& i = mat;
        return ipl2QImage(&i);
    }

    /**
     * @brief mat2QImage converts an OpenCV image to a Qt image
     * @param iplImg OpenCV
     * @return Qt image image
     */
    static TargetClass ipl2QImage(const IplImage *iplImg) {
        switch(iplImg->depth) {
        case IPL_DEPTH_8U:
            return ipl2QImageImpl<unsigned char>(iplImg);
        case (int) IPL_DEPTH_8S:
            return ipl2QImageImpl<char>(iplImg);
        case IPL_DEPTH_16U:
            return ipl2QImageImpl<unsigned short>(iplImg);
        case (int) IPL_DEPTH_16S:
            return ipl2QImageImpl<short>(iplImg);
        case (int) IPL_DEPTH_32S:
            return ipl2QImageImpl<int>(iplImg);
        case IPL_DEPTH_32F:
            return ipl2QImageImpl<float>(iplImg);
        case IPL_DEPTH_64F:
            return ipl2QImageImpl<double>(iplImg);
        default:
            throw std::runtime_error("cannot convert image, unknown type");
        }
    }

    template <typename T>
    static TargetClass ipl2QImageImpl(const IplImage *iplImg) {
        int h = iplImg->height;
        int w = iplImg->width;
        int channels = iplImg->nChannels;
        TargetClass qimg(w, h, TargetClass::Format_ARGB32);
        T* data = (T*) (iplImg->imageData);

        assert(w > 0);
        assert(h > 0);
        assert(data != NULL);

        for (int y = 0; y < h; y++, data += iplImg->widthStep / sizeof(T)) {
            for (int x = 0; x < w; x++) {
                char r = 0, g = 0, b = 0, a = 0;
                if (channels == 1) {
                    r = data[x * channels];
                    g = r;
                    b = r;
                } else if (channels == 3 || channels == 4) {
                    r = data[x * channels + 2];
                    g = data[x * channels + 1];
                    b = data[x * channels];
                }

                if (channels == 4) {
                    a = data[x * channels + 3];
                    qimg.setPixel(x, y, RGBConverter::rgba(r, g, b, a));
                } else {
                    qimg.setPixel(x, y, RGBConverter::rgb(r, g, b));
                }
            }
        }
        return qimg;
    }
};

}

#endif // IMAGE_CONVERTER_H
