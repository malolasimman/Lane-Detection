/* 
 * file name    : Lanedetector.cpp
 * Hardware     : Raspberry Pi 3B
 * project name : Real-time Lane Detection and Blindspot
                  monitoring for Enhanced Semi-Autonomous Vehicle Safety 
 * author       : Malola Simman Srinivasan Kannan, Shrinithi Venkatesan, Sriraj Vemparala
 * Reference    :https://github.com/ChipHunter/LaneDetection_with_openCV/tree/master/Project1/src
 */


#include "laneDetector.h"
#include <vector>
#include <stdexcept>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <semaphore.h>

extern int dotted;
extern int solid;
extern int left_flag;
extern int right_flag;
extern int thd1flag;      
extern sem_t sem_blind_spot;

laneDetector::laneDetector(std::string path) {
    
    _cap.open(path);

    if (!_cap.isOpened())
        throw std::runtime_error("Problem openning the video!");
        }


void laneDetector:: Noise_reduction(cv::Mat& in, cv::Mat& out) {

cv::GaussianBlur(in, out, cv::Size(3, 3), 0, 0);
}

void laneDetector::mask_requiredlane(cv::Mat& in, cv::Mat& out) {

    
    cv::Mat mask = cv::Mat::zeros(in.size(), in.type());
    cv::Point pts[4] = {
        cv::Point(150, 720),
        cv::Point(530, 500),
        cv::Point(700, 500),
        cv::Point(1100, 720)
    };

    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));

    cv::bitwise_and(in, mask, out);
}


void laneDetector::houghLines(cv::Mat& in, std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& solidlines, std::vector<cv::Vec4i>& dottedlines) {

       HoughLinesP(in, lines, 1, CV_PI / 180, 20, 20,30);
    solidlines.clear();  
    dottedlines.clear();
    int center = in.cols / 2;
    int min_line_length = 30;
    
    for (const cv::Vec4i& line : lines) 
    {
        double slope = static_cast<double>(line[3] - line[1]) / static_cast<double>(line[2] - line[0]);
        double length = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3])); 
        double dashed_threshold = 2;
        if (length < min_line_length) 
        {
            continue;
        }
        if (slope < 0 && line[2] < center && line[0] < center) 
        {
             // Classify dashed or solid line based on length
            if (length < dashed_threshold * min_line_length) {
                dottedlines.push_back(line);
                dotted = 1;
                solid = 0;
            } else {
                solidlines.push_back(line);
                solid = 1;
                dotted = 0;
            }
         
        } 
        else if (slope > 0 && line[2] > center && line[0] > center) 
        {
            // Classify dashed or solid line based on length
            if (length < dashed_threshold * min_line_length) {
                dottedlines.push_back(line);
                dotted = 1;
                solid = 0;
                if(thd1flag ==1){
                sem_post(&sem_blind_spot);
                thd1flag=0;
                }
            } else {
                solidlines.push_back(line);
                solid = 1;
                dotted = 0;
                if(thd1flag ==1){
                sem_post(&sem_blind_spot);
                thd1flag=0;
                }
            }
        }
       
    }

}

void laneDetector::readFrame(cv::Mat& frame) {

   if (!_cap.read(frame))
        throw std::runtime_error("Problem reading a frame");

}

void laneDetector::plot(std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& solidlines, std::vector<cv::Vec4i>& dottedlines,cv::Mat& frame) {

  
   for (size_t i = 0; i < solidlines.size(); i++)
        {
            cv::Vec4i l = solidlines[i];
            cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 5, CV_AA);
        
        }
        for (size_t i = 0; i < dottedlines.size(); i++)
        {
            cv::Vec4i k = dottedlines[i];
            cv::line(frame, cv::Point(k[0], k[1]), cv::Point(k[2], k[3]), cv::Scalar(0, 255,0), 5, CV_AA);
        
        }
   
       
    cv::imshow("Result Image", frame);

}

void laneDetector::edgeDetection(cv::Mat& in, cv::Mat& out) {

     cv::Mat kernel;
    cv::Point anchor;

    cv::cvtColor(in, out, cv::COLOR_RGB2GRAY);

    cv::threshold(out, out, 140, 255, cv::THRESH_BINARY);

    anchor = cv::Point(-1, -1);
    kernel = cv::Mat(1, 3, CV_32F);
    kernel.at<float>(0, 0) = -1;
    kernel.at<float>(0, 1) = 0;
    kernel.at<float>(0, 2) = 1;

    cv::filter2D(out, out, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);
}
