#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <utils/LibParticleFilter/pfilter.h>
#include <iostream>
#include <math.h>
#include <string>

#include "terra_mat.h"

using namespace cv;

Mat getRotatedCrop(const Mat& src, Point2f pos, Size size, float angle, float scale) {
    pos.x = (int)pos.x;
    pos.y = (int)pos.y;
    const RotatedRect rect(pos, size, angle);

    // create bounding box of rotated and unrotated rectangle
    Size2f boundingBoxSize(std::max((int)(rect.boundingRect().width/scale+0.5f),  size.width),
                           std::max((int)(rect.boundingRect().height/scale+0.5f), size.height));
    Rect boundingBox = RotatedRect(pos, boundingBoxSize, 0).boundingRect();

    //Point a cv::Mat header at it (no allocation is done)
    Mat cropped(src, boundingBox);

    // center relative to cropped region
    Point2f boundingBoxCenter(boundingBox.width/2.0f, boundingBox.height/2.0f);

    // get the rotation matrix
    Mat M = getRotationMatrix2D(boundingBoxCenter, rect.angle, scale);

    // perform the affine transformation
    Mat rotated;
    warpAffine(cropped, rotated, M, cv::Size(cropped.cols, cropped.rows), INTER_NEAREST);

    // crop the resulting image
    Rect outRect = RotatedRect(boundingBoxCenter, Size2f(size.width-1, size.height-1), 0).boundingRect();
    Mat cropped2(rotated, outRect);

    return cropped2;
}

float match(Mat queryImage, Mat map, Point2f pos, float angle, float scale) {
    Mat result, result2;
    Mat cropped = getRotatedCrop(map, pos, Size2f(queryImage.cols, queryImage.rows), angle, scale);

    matchTemplate(cropped, queryImage, result, CV_TM_SQDIFF);

/*//
    Mat diff = cropped - queryImage;
    Mat sq = diff.mul(diff);
    cv::Scalar summe = cv::sum(sq);
    std::cout << cv::sum(summe)[0]*70 << " " << result.at<float>(0,0) << std::endl;
//*/

/*//
    cv::subtract(cropped, queryImage, result2);
    cv::multiply(result2, result2, result2);
    std::cout << cv::sum(result2) << " " << summe << " " << result.at<float>(0,0) << std::endl;
//*/

//    std::cout << result.rows << " " << result.cols << std::endl;
/*//
    namedWindow("Display cropped", CV_WINDOW_AUTOSIZE);
    imshow("Display cropped", cropped);
//*/

//*//
    return result.at<float>(0,0) / (queryImage.cols * queryImage.rows); /*/
    return cv::sum(summe)[0]*60 / (queryImage.cols * queryImage.rows);
//*/
}

class MyParticleFilter: public ParticleFilter
{
private:
    Mat templat;
    Mat src;

public:
    MyParticleFilter(Mat src,
                     float xmin, float xmax,
                     float ymin, float ymax,
                     float zmin, float zmax,
                     Angle oriZleft, Angle oriZright,
                     unsigned int numParticles, float diffuseTrans = 1.0f)
        :
          ParticleFilter(xmin, xmax,
                         ymin, ymax,
                         zmin, zmax,
                         oriZleft, oriZright,
                         numParticles, diffuseTrans),
          src(src)
    {
    }

    void setTemplate(Mat templat) {
        this->templat = templat;
    }

    virtual double probfunc(Particle* particle) const {
        float dist = match(templat,
                           src,
                           Point2f(particle->state.posX, particle->state.posY),
                           particle->state.oriZ.getDegrees(),
                           particle->state.posZ) / 10000;
        return exp(-dist*dist);
    }
};

double dist(Point2f a, Point2f b) {
    return sqrt(pow(a.x-b.x, 2.0) + pow(a.y - b.y, 2.0));
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: command <input-file>\n");
        return -1;
    }


    std::vector<std::string> filelist;

    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 100.062.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 100.418.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 100.774.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 101.130.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 101.488.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 101.844.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 102.201.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 102.557.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 102.914.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 103.269.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 103.625.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 104.010.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 104.366.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 104.722.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 105.079.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 105.434.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 105.791.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 106.147.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 106.508.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 106.865.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 107.231.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 117.519.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 117.876.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 118.232.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 118.588.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 118.944.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 119.300.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 119.656.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 120.013.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 120.372.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 120.727.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 121.083.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 121.438.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 121.795.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 122.150.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 122.506.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 122.885.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 123.241.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 123.596.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 123.952.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 124.308.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 124.664.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 125.019.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 125.376.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 125.732.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 126.090.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 126.448.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 126.804.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 127.160.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 127.541.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 127.897.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 128.253.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 128.608.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 128.965.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 129.320.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 129.676.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 130.032.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 130.387.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 130.744.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 131.099.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 131.456.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 131.812.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 132.168.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 132.549.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 132.906.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 133.263.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 133.619.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 133.976.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 134.332.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 134.688.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 135.043.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 135.398.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 135.753.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 136.109.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 136.465.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 136.820.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 137.176.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 137.561.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 137.917.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 138.272.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 138.629.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 138.985.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 139.342.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 139.698.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 140.055.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 140.412.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 140.770.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 141.125.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 141.481.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 141.837.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 142.192.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 142.575.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 142.942.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 149.501.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 149.863.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 150.228.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 150.593.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 150.959.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 151.322.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 151.685.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 152.050.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 152.417.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 152.780.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 153.141.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 153.498.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 153.855.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 154.237.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 154.595.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 154.951.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 155.307.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 155.663.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 156.019.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 156.375.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 156.731.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 157.086.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 157.441.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 157.797.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 158.154.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 158.510.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 158.866.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 159.248.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 159.605.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 159.960.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 160.317.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 160.672.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 161.030.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 161.387.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 161.745.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 162.101.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 162.457.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 162.813.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 163.168.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 163.524.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 163.881.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 164.264.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 164.620.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 164.976.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 165.331.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 165.688.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 166.043.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 166.399.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 166.756.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 167.113.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 167.469.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 167.826.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 168.182.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 168.541.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 168.897.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 169.281.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 169.637.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 169.993.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 170.349.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 170.705.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 171.060.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 171.416.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 171.772.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 172.127.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 172.484.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 172.841.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 173.197.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 173.553.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 173.913.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 174.297.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 174.653.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 175.026.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 175.390.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 175.755.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 176.111.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 176.466.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 176.822.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 177.178.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 177.532.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 177.889.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 178.245.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 178.601.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 179.013.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 179.369.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 179.725.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 180.129.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 180.496.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 187.162.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 187.519.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 187.876.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 188.233.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 188.590.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 188.947.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 189.304.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 189.659.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 190.015.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 190.372.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 190.728.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 191.083.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 191.438.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 191.813.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 192.169.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 192.525.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 192.881.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 193.238.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 193.594.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 193.951.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 194.307.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 194.663.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 195.019.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 195.375.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 195.731.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 196.089.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 196.444.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 196.823.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 197.179.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 197.535.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 197.891.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 198.246.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 198.602.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 198.959.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 199.314.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 199.670.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 200.025.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 200.381.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 200.737.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 201.093.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 201.448.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 201.834.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 202.189.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 202.546.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 202.905.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 203.259.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 203.614.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 203.970.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 204.326.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 204.683.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 205.039.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 205.395.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 205.750.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 206.106.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 206.462.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 206.841.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 207.208.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 207.572.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 207.939.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 208.298.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 208.655.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 209.011.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 209.367.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 209.724.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 210.080.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 210.438.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 210.793.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 211.149.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 211.532.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 211.905.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 212.290.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 212.652.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 218.747.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 219.103.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 219.459.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 219.814.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 220.171.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 220.526.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 220.882.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 221.238.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 221.593.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 221.949.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 222.306.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 222.662.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 223.019.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 223.375.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 223.934.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 224.293.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 224.649.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 225.005.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 225.360.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 225.717.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 226.072.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 226.428.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 226.784.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 227.139.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 227.495.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 227.852.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 228.208.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 228.591.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 228.947.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 229.304.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 229.659.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 230.015.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 230.374.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 230.731.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 231.086.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 231.441.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 231.797.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 232.154.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 232.510.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 232.866.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 233.221.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 233.603.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 233.959.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 234.315.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 234.671.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 235.027.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 235.382.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 235.739.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 236.095.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 236.452.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 236.808.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 237.165.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 237.523.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 237.879.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 238.235.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 238.628.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 238.989.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 239.347.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 239.712.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 240.074.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 240.429.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 240.785.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 241.141.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 241.496.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 241.852.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 242.208.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 242.566.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 242.922.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 243.279.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 243.705.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 248.201.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 248.784.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 249.140.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 249.497.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 249.854.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 250.209.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 250.566.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 250.921.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 251.279.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 251.635.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 251.990.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 252.346.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 252.701.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 253.057.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 253.414.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 253.784.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 254.139.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 254.496.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 254.851.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 255.208.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 255.565.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 255.920.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 256.277.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 256.634.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 256.992.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 257.349.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 257.705.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 258.067.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 258.450.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 258.806.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 259.164.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 259.519.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 259.875.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 260.237.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 260.592.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 260.948.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 261.306.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 261.661.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 262.019.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 262.375.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 262.731.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 263.089.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 263.474.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 263.833.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 264.189.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 264.546.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 264.906.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 265.261.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 265.616.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 265.972.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 266.332.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 266.688.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 267.046.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 267.401.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 267.757.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 268.113.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 268.495.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 268.863.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 269.223.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 269.585.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 269.954.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 270.311.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 270.670.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 271.025.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 271.382.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 271.743.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 272.098.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 272.456.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 272.812.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 273.168.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 273.554.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 273.957.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 279.551.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 279.907.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 280.263.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 280.622.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 280.979.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 281.334.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 281.690.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 282.046.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 282.403.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 282.760.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 283.116.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 283.474.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 283.831.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 284.215.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 284.573.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 284.929.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 285.285.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 285.645.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 286.001.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 286.357.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 286.714.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 287.070.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 287.425.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 287.782.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 288.138.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 288.494.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 288.851.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 289.232.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 289.587.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 289.944.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 290.300.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 290.656.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 291.015.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 291.371.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 291.728.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 292.087.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 292.444.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 292.801.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 293.156.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 293.512.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 293.870.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 294.252.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 294.607.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 294.965.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 295.321.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 295.677.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 296.032.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 296.391.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 296.747.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 297.104.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 297.461.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 297.817.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 298.175.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 298.532.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 298.889.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 299.275.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 299.631.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 299.999.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 300.357.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 300.726.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 301.094.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 301.450.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 301.806.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 302.164.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 302.520.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 302.877.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 303.233.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 303.590.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 303.946.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 304.331.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 304.686.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 305.094.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 310.487.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 310.842.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 311.198.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 311.555.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 311.913.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 312.271.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 312.628.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 312.985.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 313.343.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 313.700.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 314.055.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 314.412.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 314.769.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 315.126.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 315.493.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 315.850.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 316.206.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 316.562.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 316.918.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 317.275.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 317.630.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 317.986.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 318.343.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 318.699.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 319.057.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 319.413.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 319.771.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 320.129.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 320.510.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 320.866.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 321.222.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 321.577.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 321.934.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 322.289.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 322.646.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 323.003.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 323.358.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 323.715.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 324.071.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 324.427.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 324.785.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 325.143.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 325.520.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 325.879.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 326.233.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 326.592.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 326.950.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 327.305.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 327.662.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 328.018.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 328.374.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 328.733.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 329.089.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 329.444.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 329.801.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 330.184.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 330.540.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 330.909.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 331.268.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 331.636.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 332.004.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 332.360.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 332.718.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 333.074.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 333.430.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 333.789.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 334.146.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 334.503.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 334.860.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 335.243.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 335.600.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 336.009.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 341.269.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 341.625.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 341.981.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 342.338.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 342.694.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 343.051.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 343.406.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 343.763.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 344.475.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 344.832.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 345.190.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 345.546.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 345.903.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 346.282.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 346.639.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 346.997.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 347.353.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 347.712.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 348.069.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 348.425.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 348.781.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 349.137.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 349.494.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 349.850.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 350.206.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 350.563.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 350.942.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 351.298.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 351.655.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 352.011.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 352.368.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 352.727.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 353.083.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 353.439.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 353.798.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 354.154.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 354.510.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 354.870.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 355.225.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 355.581.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 355.968.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 356.324.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 356.681.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 357.037.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 357.394.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 357.750.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 358.106.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 358.462.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 358.819.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 359.174.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 359.530.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 359.888.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 360.244.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 360.602.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 360.985.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 361.351.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 361.711.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 362.073.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 362.442.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 362.806.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 363.162.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 363.520.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 364.231.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 364.588.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 364.944.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 365.300.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 365.657.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 366.058.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 366.457.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 372.407.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 372.765.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 373.121.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 373.477.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 373.836.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 374.192.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 374.548.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 374.906.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 375.264.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 375.620.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 375.976.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 376.334.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 376.691.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 377.074.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 377.430.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 377.787.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 378.143.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 378.499.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 378.857.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 379.213.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 379.571.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 379.927.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 380.282.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 380.639.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 380.997.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 381.354.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 381.710.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 382.096.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 382.455.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 382.811.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 383.168.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 383.523.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 383.879.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 384.235.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 384.592.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 384.948.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 385.305.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 385.661.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 386.017.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 386.374.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 386.730.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 387.114.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 387.471.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 387.828.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 388.186.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 388.545.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 388.901.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 389.258.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 389.616.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 389.971.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 390.329.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 390.683.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 391.040.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 391.395.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 391.755.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 392.142.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 392.497.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 392.866.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 393.229.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 393.595.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 393.959.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 394.315.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 394.671.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 395.029.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 395.386.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 395.742.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 396.100.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 396.456.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 396.813.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 397.372.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 397.728.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 398.128.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  82.175.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  82.530.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  82.886.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  83.242.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  83.597.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  83.952.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  84.330.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  84.687.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  85.043.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  85.398.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  85.753.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  86.108.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  86.464.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  86.819.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  87.174.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  87.531.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  87.887.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  88.242.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  88.598.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  88.953.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  89.327.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  89.683.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  90.039.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  90.396.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  90.753.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  91.109.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  91.466.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  91.823.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  92.179.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  92.536.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  92.893.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  93.249.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  93.605.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  93.960.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  94.335.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  94.691.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  95.047.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  95.403.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  95.759.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  96.116.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  96.472.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  96.828.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  97.183.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  97.540.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  97.897.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  98.254.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  98.609.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  98.992.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  99.350.jpg.640.jpg.yml");
    filelist.push_back("/localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog-  99.706.jpg.640.jpg.yml");

    TerraMat mapTest;
    mapTest.read("/localhome/masselli/svn/rabot/trunk/Utils/andreas/mapstest_cropped.jpg.yml");

    TerraMat testTemplate;

//  /localhome/masselli/svn/rabot/trunk/Utils/andreas/terra_undist_640/undistlog- 100.774.jpg.640.jpg.yml

    Mat src;
/*//
    src = imread(argv[1], 1); /*/
    src = mapTest.getBGR();
//*/

    std::cout << "Map: " << src.cols << "x" << src.rows << std::endl;

    if(!src.data) {
        printf("No image data \n");
        return -1;
    }

    int sx = src.cols/6;//argc < 3 ? 2300 : atoi(argv[2]);
    int sy = src.rows/3;//argc < 4 ? 1400 : atoi(argv[3]);
    int w = std::min(src.cols/2, 300);
    int h = std::min(src.rows/2, 300);

    Point2f pos(sx+75, sy+75);

//    std::cout << match(templat, src, pos,  0) << std::endl;
//    std::cout << match(templat, src, pos, 10) << std::endl;
//    std::cout << match(templat, src, pos,-10) << std::endl;
//    std::cout << match(templat, src, pos, 20) << std::endl;
//    std::cout << match(templat, src, pos,-20) << std::endl;

//*////////////////

    MyParticleFilter pfilter(src,
                             sx, sx+w,
                             sy, sy+h,
                             1.3f, 2.7f,
                             Angle(20, 1), Angle(-80, 1),
                             360, 0.3f);

    for (int i = 0; i < 6000; ++i) {
//        pos.x += 0.1;
  //      pos.y += 0.1;
//            Mat templat = src(Rect(pos.x-64/2+1, pos.y-48/2+1, 64, 48));

/*//
        Mat templat = getRotatedCrop(src, pos, Size(64, 48), -45, 2.5f); /*/
        testTemplate.read(filelist[i/10]);
        Mat templat = testTemplate.getBGR();
            templat = templat(Rect(17/1.5625, 13/1.5625, 64/1.5625, 48/1.5625));
//*/

//*//
        namedWindow("Display templat", CV_WINDOW_AUTOSIZE);
        imshow("Display templat", templat);
        //moveWindow("Display templat", templat.cols+64, 0);
//*/

        pfilter.setTemplate(templat);
        pfilter.update();

        Pose meanPose = pfilter.getMean(3).state;
        std::cout << "meanPose.posZ " << meanPose.posZ
                  << "\tmeanPose.oriZ " << meanPose.oriZ.getDegrees() <<  std::endl;

        // output
/*//        cout.precision(5);
        for (int j=0; j < pfilter.getParticles().size(); ++j) {
            std::cout << pfilter.getParticles()[j].state.posX << "\t";
            std::cout << pfilter.getParticles()[j].state.posY << "\n";
        }
        std::cout << std::endl;
//*/
        // debug output
//            std::cout << "." << std::flush;

        //*//
        Mat display(src(Rect(sx, sy, w, h)).clone());
        for (int j=0; j < pfilter.getParticles().size(); ++j) {
            //std::cout << pfilter.getParticles()[j].state.posX << "\t";
            //std::cout << pfilter.getParticles()[j].state.posY << "\n";
            cv::circle(display,
                     cvPoint(pfilter.getParticles()[j].state.posX-sx,
                             pfilter.getParticles()[j].state.posY-sy),
                     1,
                     cvScalar(255, 128, 0),
                     2);
        }

        // show mean pose
        cv::circle(display,
                 cvPoint(meanPose.posX-sx,
                         meanPose.posY-sy),
                 1,
                 cvScalar(0, 128, 255),
                 2);

/*/        // show ground truth pose
        cv::circle(display,
                 cvPoint(pos.x-sx,
                         pos.y-sy),
                 1,
                 cvScalar(0, 0, 255),
                 2);

 //           std::cout << dist(pos, Point2f(meanPose.posX, meanPose.posY)) << std::endl;
*/
        namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
        imshow("Display Image", display);
        //moveWindow("Display Image", 0, templat.rows+64);
    //*/
        if (waitKey(1) != -1)
            break;
    }
//*////////////////

/*  // test terramat
    const int noOfClasses = 4;
    cv::Mat  terrain(480, 640, CV_32FC(noOfClasses), cv::Scalar::all(0));
    TerraMat terraMatrix(terrain);
    terraMatrix.addLegendEntry(TerrainClass(0, "Grass", cv::Vec3b(  0, 255, 255)));
    terraMatrix.addLegendEntry(TerrainClass(1, "Bush",  cv::Vec3b(255,   0,   0)));
    terraMatrix.addLegendEntry(TerrainClass(2, "Road",  cv::Vec3b(128, 128, 128)));
    terraMatrix.addLegendEntry(TerrainClass(3, "Roof",  cv::Vec3b(  0,   0, 255)));
//    terraMatrix.addTerrainClass(TerrainClass(4, "Other", cv::Vec3b(255, 255, 255)));

    terraMatrix.at< Vec<float, noOfClasses> >(3,4)[1] = 256.3;
    terraMatrix.at< Vec<float, noOfClasses> >(5,4)[3] = 256.3;

    terraMatrix.write("test.yml");
    terraMatrix.read("test.yml");

    std::cout << terraMatrix.at< Vec<float, noOfClasses> >(3,4)[0] << " "
//              << rgbMatrix.at< Vec<float, noOfClasses> >(3,4)[4]
              << (int)terraMatrix.getFavorites().at<uchar>(5,4) << " "
            << std::endl;

    namedWindow("Display rgbMatrix", CV_WINDOW_AUTOSIZE);
    imshow("Display rgbMatrix", terraMatrix.getFavoritesBGR());
    waitKey(0);

    imshow("Display rgbMatrix", terraMatrix.getBGR());
    waitKey(0);
*/

    // create match matrix (brute force match)
    Mat matchMatrix = Mat::zeros(h, w, CV_32FC(1));
//*//
    src = mapTest.getBGR();
/*//
    Mat template4Matrix = getRotatedCrop(src, pos, Size(64, 48), -45, 2.0f); /*/
    Mat template4Matrix = testTemplate.getBGR();
//*/

    for (int x = sx; x < sx+w; ++x) {
        for (int y = sy; y < sy+h; ++y) {
            float dist = match(template4Matrix, src, Point2f(x, y), -27, 2.5f) / 15000;
            matchMatrix.at< cv::Vec<float, 1> >(y-sy, x-sx)[0] = exp(-dist);
        }
        std::cout << (int)(((float)(x-sx))/w*100) << "%" << std::endl;
        if (waitKey(1) != -1)
            break;
    }

    namedWindow("Display matchMatrix", CV_WINDOW_AUTOSIZE);
    imshow("Display matchMatrix", matchMatrix);
    moveWindow("Display matchMatrix", 0, template4Matrix.rows+64);
//*/

/*//
    for (int x = sx; x < sx+w; ++x)
        for (int y = sy; y < sy+h; ++y) {
            Vec<float, 1> pix = matchMatrix.at< Vec<float, 1> >(y-sy, x-sx);
            std::cout << pix[0] << std::endl;
        }
//*/

    waitKey(0);

    return 0;
}


