#include <iostream>

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>

// 1. Camera instrinsic and disortotion matrix
// 2.


void readCameraParameters(const std::string& pathCamParam){

}



int main() {
    cv::FileStorage fileParam("/home/oguz/Desktop/relimetrics/intrinsics.yml", cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;
    if(fileParam.isOpened()){
        fileParam["M"] >> cameraMatrix;
        fileParam["D"] >> distCoeffs;
    }else{
        std::cerr << "fileParam not found " << std::endl;
    }

    // Turn into parameter
    cv::Size patternSize(10,7) ;
    std::vector<cv::Point2f> corners;
    cv::Mat image = cv::imread("/home/oguz/Desktop/relimetrics/ChessBoard_Centered.tiff", cv::IMREAD_GRAYSCALE);
    if(image.empty())
        throw std::runtime_error("Image not found");
    /*
    for (int i = 12; i > 3; --i) {
        for (int j = 12; j >3 ; --j) {
            cv::Size patternSize(i,j) ;
            bool imagePointsFound = cv::findChessboardCorners(image, patternSize, corners);
            if (imagePointsFound)
                std::cout << "i:" << i << "j: " << j << std::endl;
        }
    }
    */

    bool imagePointsFound = cv::findChessboardCorners(image, patternSize, corners);
    if (!imagePointsFound)
        throw std::runtime_error(" Image Points not found");

    //cv::drawChessboardCorners(image,patternSize,corners,imagePointsFound);
    //cv::resize(image, image, cv::Size(image.cols * 0.25, image.rows * 0.25), 0, 0);
    //cv::imshow("o",image);
    //cv::waitKey(0);


    cv::cornerSubPix(image,corners,
            cv::Size(5,5),
            cv::Size(-1,-1),
            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

    //cv::Mat* objectPointsPointer = new cv::Mat(cv::Size(patternSize.width, patternSize.height), CV_64FC1);
    std::vector<cv::Point3f> objectPoints;


    //objectPoints.col(0).row(0) = (1364.5, -27.2);
    float xAxes = 1364.5 + 40;
    float yAxes = -27.2 + 40;
    float zAxes = 1200;
    if(corners.size() == patternSize.area())
    {
        for(int i = 0; i < patternSize.width; i++){
            for(int j = 0; j < patternSize.height; j++){
                cv::Point3f p = {(xAxes+i*20),(yAxes+j*20),zAxes};
                objectPoints.emplace_back(p);
            }
        }
    }
    //std::cout << objectPoints << std::endl;
    cv::Vec3f rotationVec,translationVec;

    // Make sure that the last flat is set true
    bool cameraExtrinsicParamFound  = cv::solvePnP(objectPoints,corners,cameraMatrix, distCoeffs,rotationVec,translationVec,false,0);

    if(!cameraExtrinsicParamFound)
        throw std::runtime_error(" Extrinsic Paramer not found");
    //std::cout << rotationVec << std::endl;
    //std::cout << translationVec << std::endl;


    // rodriguez
    cv::Matx33f rotationMatrix;
    cv::Rodrigues(rotationVec,rotationMatrix);

    //Homogenous tranformation
    cv::Affine3f extrinsicParam(rotationMatrix, translationVec);
    std::cout << extrinsicParam.matrix << std::endl;

    // Inverse
    cv::Affine3f extrinsicParam_inv;
    extrinsicParam_inv = extrinsicParam.inv();
    std::cout << extrinsicParam_inv.matrix << std::endl;







    return 0;
}
