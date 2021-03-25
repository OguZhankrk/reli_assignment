#include <iostream>

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include "cameraPoseEstimation.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>


#include <boost/program_options.hpp>






int main(int argc, char **argv) {
    /*
    std::string pathToCameraFile, pathToChessBoardImage;
    cv::Size patchSize,patternSize;
    cv::Point3f worldPoint;
    double distance;

    std::cout << "Enter the path to camera File: ";
    std::cin >> pathToCameraFile;
    std::cout << "Enter the path to chessBoardImage: ";
    std::cin >> pathToChessBoardImage;
    std::cout << "Enter first width then height of patch size in mm: ";
    std::cin >> patchSize.width >>patchSize.height;
    std::cout << "Enter first width and then height of pattern on chess board: ";
    std::cin >> patternSize.width >> patternSize.height;
    std::cout << "Enter  first the object point x then y: ";
    std::cin >> worldPoint.x >> worldPoint.y;
    std::cout << "Enter distance from target chess board to the camera center in mm: ";
    std::cin >> distance;



    cameraPoseEstimation c(pathToCameraFile,patternSize);
    std::vector<cv::Point2f>  corners = c.findBoardCorners(pathToChessBoardImage);
    std::vector<cv::Point3f> objectPoints = c.findObjectPoints(worldPoint);
    c.findExtrinsicParam(corners,objectPoints);
    cv::Affine3f extrinsicParam_inv = c.getHomogenousMatrix();
    */






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


    bool imagePointsFound = cv::findChessboardCorners(image, patternSize, corners);
    if (!imagePointsFound)
        throw std::runtime_error(" Image Points not found");

    /*
    cv::drawChessboardCorners(image,patternSize,corners,imagePointsFound);
    cv::resize(image, image, cv::Size(image.cols * 0.25, image.rows * 0.25), 0, 0);
    cv::imshow("o",image);
    cv::waitKey(0);
    */

    std::cout << corners<< std::endl;


    cv::cornerSubPix(image,corners,
            cv::Size(5,5),
            cv::Size(-1,-1),
            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

    //cv::Mat* objectPointsPointer = new cv::Mat(cv::Size(patternSize.width, patternSize.height), CV_64FC1);
    std::vector<cv::Point3f> objectPoints;

    // try 4 Corners one by one
    //float xAxes = 1364.5 - 220;
    //float yAxes = -27.2 + 160;
    //float zAxes = 0;

    //float xAxes = 1364.5 + 160;
    //float yAxes = -27.2 - 220;
    //float zAxes = 0;

    float xAxes = 1364.5 - 220;
    float yAxes = -27.2 - 160;
    float zAxes = 0;

    if(corners.size() == patternSize.area())
    {
        for(int i = 0; i < patternSize.height; i++){
            for(int j = 0; j < patternSize.width; j++){
                cv::Point3f p = {(xAxes + j*20),(yAxes + i*20),1200};
                objectPoints.emplace_back(p);
            }
        }
    }
    std::cout << objectPoints << std::endl;

    cv::Vec3f rotationVec,translationVec;

    // Make sure that the last flat is set true
    bool cameraExtrinsicParamFound  = cv::solvePnP(objectPoints,corners,cameraMatrix, distCoeffs,rotationVec,translationVec,false,0);


    //Check if the object points are true
    std::vector<cv::Point2f> imagePoints;
    //cv::projectPoints(objectPoints,rotationVec,translationVec,cameraMatrix,distCoeffs,imagePoints);


    cv::Mat image2 = cv::imread("/home/oguz/Desktop/relimetrics/ChessBoard_Centered.tiff", cv::IMREAD_GRAYSCALE);
    for (auto p : imagePoints) {
        cv::circle(image2,p,8,cv::Scalar(195,0,255),  -1);
        break;
    }
    /*
    cv::resize(image2, image2, cv::Size(image2.cols * 0.25, image2.rows * 0.25), 0, 0);
    cv::imshow("o",image2);
    cv::waitKey(0);
    */

    if(!cameraExtrinsicParamFound)
        throw std::runtime_error(" Extrinsic Parameter not found");
    std::cout << rotationVec << std::endl;
    std::cout << translationVec << std::endl;


    // rodriguez
    cv::Matx33f rotationMatrix;
    cv::Rodrigues(rotationVec,rotationMatrix);

    //Homogenous tranformation
    cv::Affine3f extrinsicParam(rotationMatrix, translationVec);
    //std::cout << extrinsicParam.matrix << std::endl;

    // Inverse
    cv::Affine3f extrinsicParam_inv;
    extrinsicParam_inv = extrinsicParam.inv();
    std::cout << extrinsicParam_inv.matrix << std::endl;

    return 0;
}
