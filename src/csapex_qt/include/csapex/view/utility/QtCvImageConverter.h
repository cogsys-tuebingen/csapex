#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <stdexcept>
#include <QImage>

namespace QtCvImageConverter
{
/**
 * @brief The QTQTRGBConverter struct is the default rgb mapper
 */
struct QTRGBConverter
{
    static inline unsigned int rgb(int r, int g, int b)  // set RGB value
    {
        return (0xffu << 24) | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff);
    }

    static inline unsigned int rgba(int r, int g, int b, int a)  // set RGBA value
    {
        return ((a & 0xff) << 24) | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff);
    }
};

static inline int getRed(unsigned rgb)
{
    return ((rgb >> 16) & 0xff);
}

static inline int getGreen(unsigned rgb)
{
    return ((rgb >> 8) & 0xff);
}

static inline int getBlue(unsigned rgb)
{
    return (rgb & 0xff);
}

static inline int getAlpha(unsigned rgb)
{
    return rgb >> 24;
}

/**
 * @brief The Converter class is a helper class template for converting QImages to cv images
 * @note It is a template so that Qt doesn't have to be linked to this library
 */
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
    static cv::Mat QImage2Mat(const QImage& img)
    {
        int h = img.height();
        int w = img.width();
        cv::Mat mat(h, w, CV_8UC3);

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                unsigned source = img.pixel(x, y);
                cv::Vec3b& target = mat.at<cv::Vec3b>(y, x);
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
    static QImage mat2QImage(const cv::Mat& mat)
    {
        switch (mat.depth()) {
            case IPL_DEPTH_8U:
                return mat2QImageImpl<unsigned char>(mat);
            case (int)IPL_DEPTH_8S:
                return mat2QImageImpl<char>(mat);
            case IPL_DEPTH_16U:
                return mat2QImageImpl<unsigned short>(mat);
            case (int)IPL_DEPTH_16S:
                return mat2QImageImpl<short>(mat);
            case (int)IPL_DEPTH_32S:
                return mat2QImageImpl<int>(mat);
            case IPL_DEPTH_32F:
                return mat2QImageImpl<float>(mat);
            case IPL_DEPTH_64F:
                return mat2QImageImpl<double>(mat);
            default:
                throw std::runtime_error("cannot convert image, unknown type");
        }
    }

    template <typename T>
    static QImage mat2QImageImpl(const cv::Mat& mat)
    {
        int h = mat.rows;
        int w = mat.cols;
        int channels = mat.channels();
        QImage qimg(w, h, QImage::Format_ARGB32);
        T* data = (T*)(mat.data);

        assert(w > 0);
        assert(h > 0);
        assert(data != NULL);

        for (int y = 0; y < h; y++, data += mat.step1() / sizeof(T)) {
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
                    qimg.setPixel(x, y, QTRGBConverter::rgba(r, g, b, a));
                } else {
                    qimg.setPixel(x, y, QTRGBConverter::rgb(r, g, b));
                }
            }
        }
        return qimg;
    }
};

}  // namespace QtCvImageConverter

#endif  // IMAGE_CONVERTER_H
