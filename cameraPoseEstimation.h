#pragma once



#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>


class cameraPoseEstimation {
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Size patternSize_;
    cv::Vec3f rotationVec;
    cv::Vec3f translationVec;

public:

    cameraPoseEstimation(const std::string& pathToCameraFile,const cv::Size& patternSize);

    ~cameraPoseEstimation();

    void loadCameraInformation(const std::string pathToCameraFile);

    std::vector<cv::Point2f> findBoardCorners( const std::string& pathToChessBoard)const;

    std::vector<cv::Point3f> findObjectPoints(const cv::Point3f& objectPoint)const;

    void findExtrinsicParam(const std::vector<cv::Point2f>& imagePoints, const std::vector<cv::Point3f>& objectPoints);

    cv::Affine3f getHomogenousMatrix()const;
};