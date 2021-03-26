

#include "cameraPoseEstimation.h"

cameraPoseEstimation::cameraPoseEstimation(const std::string& pathToCameraFile, const cv::Size& patternSize)
: patternSize_(patternSize)
{
    loadCameraInformation(pathToCameraFile);

}
cameraPoseEstimation::~cameraPoseEstimation()
{
    std::cout << " object is destroyed" << std::endl;
}


void cameraPoseEstimation::loadCameraInformation(const std::string pathToCameraFile) {
    cv::FileStorage fileParam(pathToCameraFile, cv::FileStorage::READ);
    if(fileParam.isOpened()){
        fileParam["M"] >> cameraMatrix;
        fileParam["D"] >> distCoeffs;
    }else{
        std::cerr << "fileParam not found " << std::endl;
    }
}

std::vector<cv::Point2f> cameraPoseEstimation::findBoardCorners(const std::string& pathToChessBoard)const {
    std::vector<cv::Point2f> corners;
    cv::Mat image = cv::imread(pathToChessBoard, cv::IMREAD_GRAYSCALE);
    if(image.empty())
        throw std::runtime_error("Image not found");
    bool imagePointsFound = cv::findChessboardCorners(image, patternSize_, corners);
    if (!imagePointsFound)
        throw std::runtime_error(" Image Points not found");
    cv::cornerSubPix(image,corners,
                     cv::Size(5,5),
                     cv::Size(-1,-1),
                     cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
    return corners;
}


std::vector<cv::Point3f> cameraPoseEstimation::findObjectPoints(const cv::Point3f& objectPoint)const {
    std::vector<cv::Point3f> objectPoints;
    float xAxes = objectPoint.x - 220; //
    float yAxes = objectPoint.y - 160;
    float zAxes = 0;
    for (int i = 0; i < patternSize_.height; i++) {
        for (int j = 0; j < patternSize_.width; j++) {
            cv::Point3f p = {(xAxes + j * 20), (yAxes + i * 20), 0};
            objectPoints.emplace_back(p);
            }
    }
    return objectPoints;
}

void cameraPoseEstimation::findExtrinsicParam(const std::vector<cv::Point2f>& imagePoints, const std::vector<cv::Point3f>& objectPoints){
    bool cameraExtrinsicParamFound  = cv::solvePnP(objectPoints,imagePoints,
            cameraMatrix, distCoeffs,
            rotationVec,translationVec,false,0);
    if(!cameraExtrinsicParamFound)
        throw std::runtime_error(" Extrinsic Parameter not found");
}


cv::Affine3f cameraPoseEstimation::getHomogenousMatrix()const{  // convert from rodriguez into rotation matrix
    cv::Matx33f rotationMatrix;
    cv::Rodrigues(rotationVec, rotationMatrix);
    cv::Affine3f extrinsicParam(rotationMatrix, translationVec);
    cv::Affine3f extrinsicParam_inv;
    extrinsicParam_inv = extrinsicParam.inv();
    std::cout << extrinsicParam_inv.matrix << std::endl;



    return extrinsicParam_inv;
}



