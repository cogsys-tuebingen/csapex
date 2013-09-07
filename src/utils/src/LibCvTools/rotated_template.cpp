#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <utils/LibParticleFilter/pfilter.h>
#include <iostream>
#include <math.h>

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

 /*   cv::subtract(cropped, queryImage, result2);
    cv::multiply(result2, result2, result2);
    std::cout << cv::sum(result2) << " " << result.at<float>(0,0) << std::endl;
*/
//    std::cout << result.rows << " " << result.cols << std::endl;
/*//
    namedWindow("Display cropped", CV_WINDOW_AUTOSIZE);
    imshow("Display cropped", cropped);
//*/

    return result.at<float>(0,0) / (queryImage.cols * queryImage.rows);
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

    TerraMat mapTest;
    mapTest.read("/localhome/masselli/svn/rabot/trunk/Utils/andreas/mapstest_cropped.jpg.yml");

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
    int sy = src.rows/6;//argc < 4 ? 1400 : atoi(argv[3]);
    int w = std::min(src.cols/2, 300);
    int h = std::min(src.rows/2, 300);

    Point2f pos(sx+7, sy+7);

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
                             Angle(-20, 1), Angle(-60, 1),
                             260, 1.0f);

    for (int i = 0; i < 6000; ++i) {
        pos.x += 0.1;
        pos.y += 0.1;
//            Mat templat = src(Rect(pos.x-64/2+1, pos.y-48/2+1, 64, 48));
        Mat templat = getRotatedCrop(src, pos, Size(64, 48), -45, 2.5f);

//*//
        namedWindow("Display templat", CV_WINDOW_AUTOSIZE);
        imshow("Display templat", templat);
        //moveWindow("Display templat", templat.cols+64, 0);
//*/

        pfilter.setTemplate(templat);
        pfilter.update();

        Pose meanPose = pfilter.getMean(13).state;
        //std::cout << "meanPose.posZ " << meanPose.posZ <<  std::endl;
        std::cout << "meanPose.oriZ " << meanPose.oriZ.getDegrees() <<  std::endl;

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

        // show ground truth pose
        cv::circle(display,
                 cvPoint(pos.x-sx,
                         pos.y-sy),
                 1,
                 cvScalar(0, 0, 255),
                 2);

 //           std::cout << dist(pos, Point2f(meanPose.posX, meanPose.posY)) << std::endl;

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
    Mat template4Matrix = getRotatedCrop(src, pos, Size(64, 48), -45, 2.0f);
    for (int x = sx; x < sx+w; ++x) {
        for (int y = sy; y < sy+h; ++y) {
            float dist = match(template4Matrix, src, Point2f(x, y), -40, 2.0f) / 15000;
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


