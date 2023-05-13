/* 
 * file name    : lanedetector.h
 * Hardware     : Raspberry Pi 3B
 * project name : Real-time Lane Detection and Blindspot
                  monitoring for Enhanced Semi-Autonomous Vehicle Safety 
 * author       : Malola Simman Srinivasan Kannan, Shrinithi Venkatesan, Sriraj Vemparala
 * Reference    : https://github.com/ChipHunter/LaneDetection_with_openCV/tree/master/Project1/src
 */
#include "opencv2/opencv.hpp"
#include <string>

class laneDetector {
public:
	laneDetector(std::string path);

	 void Noise_reduction(cv::Mat& in, cv::Mat& out);
	 void edgeDetection(cv::Mat& in, cv::Mat& out);
	 void mask_requiredlane(cv::Mat& in, cv::Mat& out);
	 void houghLines(cv::Mat& in, std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& solidlines, std::vector<cv::Vec4i>& dottedlines);
	 void readFrame(cv::Mat& frame);
	 void plot(std::vector<cv::Vec4i>& lines,  std::vector<cv::Vec4i>& solidlines, std::vector<cv::Vec4i>& dottedlines,cv::Mat& frame);

private:
	cv::VideoCapture _cap;

};


