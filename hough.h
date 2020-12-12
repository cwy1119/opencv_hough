#ifndef HOUGH_H
#define HOUGH_H

#include "opencv2/core/core.hpp"
#include <vector>


/**
 * @author: cwy
 * @date: 2020/12/10
 * 利用hough变换提取直线, 圆等几何形状
 * 依赖opencv库
*/
class Hough{
private:
    const double DEG2RAD = 0.017453293f;
    const int MINR = 40;
    const int MASK = 5;

    cv::Mat image;
    cv::Mat accu; //参数空间
    cv::Mat getLinesSpace();
    cv::Mat getCirclesSpace();
public:
    Hough();
    ~Hough();
    void setImage(cv::Mat image); 
    cv::Mat getLinesSpaceImage();      
    cv::Mat getCirclesSpaceImage();      
    std::vector<std::pair<cv::Point,cv::Point> > getLines(int threshold = 0);
    std::vector<std::pair<cv::Point,int> > getCircles();
};

#endif