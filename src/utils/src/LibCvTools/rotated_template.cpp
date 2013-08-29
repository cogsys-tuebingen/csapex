#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <utils/LibParticleFilter/pfilter.h>
#include <iostream>
#include <math.h>

#include "terra_mat.h"

using namespace cv;

Mat getRotatedCrop(Mat src, Point2f pos, Size size, float angle) {
    const RotatedRect rect(pos, size, angle);

    // create bounding box of rotated and unrotated rectangle
    Size2f boundingBoxSize(std::max(rect.boundingRect().width,  size.width),
                           std::max(rect.boundingRect().height, size.height));
    Rect boundingBox = RotatedRect(pos, boundingBoxSize, 0).boundingRect();

    //Point a cv::Mat header at it (no allocation is done)
    Mat cropped = src(boundingBox);

    // center relative to cropped region
    Point2f boundingBoxCenter(boundingBox.width/2.0f, boundingBox.height/2.0f);

    // get the rotation matrix
    Mat M = getRotationMatrix2D(boundingBoxCenter, rect.angle, 1.0);

    // perform the affine transformation
    Mat rotated;
    warpAffine(cropped, rotated, M, boundingBoxSize, INTER_NEAREST);

    // crop the resulting image
    getRectSubPix(rotated, size, boundingBoxCenter, cropped);

    return cropped;
}

float match(Mat queryImage, Mat map, Point2f pos, float angle) {
    Mat result, result2;
    Mat cropped = getRotatedCrop(map, pos, Size2f(queryImage.cols, queryImage.rows), angle);

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
                     float xmin, float xmax, float ymin, float ymax,
                   unsigned int numParticles, float diffuseTrans = 1.0f)
        :
          ParticleFilter(xmin, xmax, ymin, ymax,
                         numParticles, diffuseTrans),
          src(src)
    {
    }

    void setTemplate(Mat templat) {
        this->templat = templat;
    }

    virtual double probfunc(Particle* particle) const {
        float dist = match(templat, src, Point2f(particle->state.posX, particle->state.posY), -45) / 10000;
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

    Mat src;
    src = imread(argv[1], 1);

    if(!src.data) {
        printf("No image data \n");
        return -1;
    }

    int w = 500;
    int h = 500;
    int sx = argc < 3 ? 2300 : atoi(argv[2]);
    int sy = argc < 4 ? 1400 : atoi(argv[3]);

    Point2f pos(sx+70, sy+70);

//    std::cout << match(templat, src, pos,  0) << std::endl;
//    std::cout << match(templat, src, pos, 10) << std::endl;
//    std::cout << match(templat, src, pos,-10) << std::endl;
//    std::cout << match(templat, src, pos, 20) << std::endl;
//    std::cout << match(templat, src, pos,-20) << std::endl;

//    Mat matchMatrix = Mat::zeros(h, w, CV_32FC(1));
/*
    for (int x = sx; x < sx+w; ++x) {
        for (int y = sy; y < sy+h; ++y) {
            float dist = match(templat, src, Point2f(x, y), 0) / 15000;
            matchMatrix.at< cv::Vec<float, 1> >(y-sy, x-sx)[0] = exp(-dist);
        }
        std::cout << (int)(((float)(x-sx))/w*100) << "%" << std::endl;
    }
//*/

/*//
    for (int x = sx; x < sx+w; ++x)
        for (int y = sy; y < sy+h; ++y) {
            Vec<float, 1> pix = matchMatrix.at< Vec<float, 1> >(y-sy, x-sx);
            std::cout << pix[0] << std::endl;
        }
//*/

//*////////////////
        MyParticleFilter pfilter(src, sx, sx+w, sy, sy+h, 260, 7.0f);

        for (int i = 0; i < 6000; ++i) {
            ++pos.x;
            ++pos.y;
//            Mat templat = src(Rect(pos.x-64/2+1, pos.y-48/2+1, 64, 48));
            Mat templat = getRotatedCrop(src, pos, Size(64, 48), -45);

        //*//
            namedWindow("Display templat", CV_WINDOW_AUTOSIZE);
            imshow("Display templat", templat);
            moveWindow("Display templat", templat.cols+64, 0);
        //*/


        pfilter.setTemplate(templat);
        pfilter.update();

        Pose meanPose = pfilter.getMean(13).state;

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
		    moveWindow("Display Image", 0, templat.rows+64);
		//*/
		    if (waitKey(1) != -1)
		        break;
		}
//*////////////////

    const int noOfClasses = 4;
    cv::Mat  terrain(480, 640, CV_32FC(noOfClasses), cv::Scalar::all(0));
    TerraMat terraMatrix(terrain);
    terraMatrix.addTerrainClass(TerrainClass(0, "Grass", cv::Vec3b(  0, 255, 255)));
    terraMatrix.addTerrainClass(TerrainClass(1, "Bush",  cv::Vec3b(255,   0,   0)));
    terraMatrix.addTerrainClass(TerrainClass(2, "Road",  cv::Vec3b(128, 128, 128)));
    terraMatrix.addTerrainClass(TerrainClass(3, "Roof",  cv::Vec3b(  0,   0, 255)));
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
    imshow("Display rgbMatrix", terraMatrix.getFavoritesRGB());

/*//
    namedWindow("Display matchMatrix", CV_WINDOW_AUTOSIZE);
    imshow("Display matchMatrix", matchMatrix);
    moveWindow("Display matchMatrix", 0, templat.rows+64);
//*/

    waitKey(0);

    return 0;
}


