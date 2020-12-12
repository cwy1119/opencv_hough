#include "hough.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
Hough::Hough(){}
Hough::~Hough(){}
/**
 * 
 * 设置Hough识别的图片
 * @param image 经过边缘检测的灰度图
 * 
*/
void Hough::setImage(cv::Mat image)
{
    this->image = image.clone();
}

/**
 * 获得经过hough变换的直线的参数空间
 * 以ρ和θ为空间的参数
 * @return cv::Mat 二维矩阵,以ρ和θ为量度
*/ 
cv::Mat Hough::getLinesSpace()
{
    int image_w = this->image.cols;
    int image_h = this->image.rows;

    int max_r = sqrt(pow((double)image_h,2)+pow((double)image_w,2))/2 + 0.5; //对角线长度的一半 R,+0.5四舍五入
    int accu_w = 180;
    int accu_h = max_r*2;  //-r~r

    // //以图像的中心为坐标原点
    double x_center = (double) image_w/2;   
    double y_center = (double) image_h/2;   
    cv::Mat accu(accu_h,accu_w,CV_32FC1,cv::Scalar::all(0));


    for(int y=0; y<image_h;y++){
        for(int x=0; x< image_w; x++){
            if(image.data[y*image_w + x] > 250)
            {
                // x*cosθ + y*sinθ = r
                for(int angle = 0; angle<accu_w; angle++){
                    double angle_rad = (double) angle * DEG2RAD;
                    int r = (double)(x-x_center)*cos(angle_rad) + (double)(y-y_center)*sin(angle_rad);
                    accu.data[(int)((r+max_r)*180.0 + angle)] += 1;
                }
            }
        }
    }
    return accu;
}

/**
 * 获取line经过hough变换的参数空间
 * 转换为以图像的形式,方便观察
*/ 
cv::Mat Hough::getLinesSpaceImage()
{
    int image_w = this->image.cols;
    int image_h = this->image.rows;

    int max_r = sqrt(pow((double)image_h,2)+pow((double)image_w,2))/2 + 0.5; //对角线长度的一半 R,+0.5四舍五入
    int accu_w = 180;
    int accu_h = max_r*2;  //-r~r

    cv::Mat accu = this->getLinesSpace();
    long max = 0;

    //获得其中最大值
    for(int y=0;y<accu_h;y++){
        for(int x=0;x<accu_w;x++){
            if(accu.data[y*accu_w+x] > max) max = accu.data[y*accu_w+x];
        }
    }

    cv::Mat result(accu_h,accu_w,CV_8UC1,cv::Scalar(0));
    for(int y=0;y<accu_h;y++){
        for(int x=0;x<accu_w;x++){
            result.data[y*accu_w+x] = (double)accu.data[y*accu_w+x]/max*255;
        }
    }
    
    return result;
}

/**
 * 获取检测到的直线
 * @param threshold 检测的阈值, 默认为图片长(宽)的四分之一
 * 返回直线端点对
 */
std::vector<std::pair<cv::Point,cv::Point> > Hough::getLines(int threshold)
{
    int image_w = this->image.cols;
    int image_h = this->image.rows;
    if(threshold == 0){
        threshold = std::max(image_h,image_w)/4;
    }
    cv::Mat accu = this->getLinesSpace();

    int w = accu.cols;
    int h = accu.rows;

    // //以图像的中心为坐标原点
    double x_center = (double) image_w/2;   
    double y_center = (double) image_h/2; 

    std::vector<std::pair<cv::Point,cv::Point> > lines;

    for(int r=0; r<h;r++){
        for(int angle =0;angle<w;angle++)
        {
            //选取投票数超过阈值的点
            if(accu.data[r*w+angle] >= threshold)
            {
                int max = accu.data[r*w+angle];
                /**
                 * 因为在参数空间中, 投票数较多的点往往是扎堆的
                 * 为了避免最后识别的线段都是密集在一起的
                 * 所以只取局域6*6范围内最大的点为有效点
                 */ 
                for(int i = -3;i<=3;i++)
                {
                    for(int j=-3;j<=3;j++){
                        //确保检测值不越界
                        if(r+i>=0 && r+i<h && angle+j>=0 && angle+j<w){
                            if( accu.data[(r+i)*w+angle+j] > max)
                            {
                                max = accu.data[(r+i)*w+angle+j];
                                i = j = 4; //找到周围点比当前点票数多, 直接跳出循环
                            }
                        }
                    } 
                }
                // 找到周围点比当前点票数多, 直接跳出循环
                if(max > (int)accu.data[r*w+angle]) continue;

                int x1,x2,y1,y2;
                //y = (r-x*cosθ)/sinθ;
                //平行的情况单独讨论
                if(angle == 0){
                    x1 = x2 = r;
                    y1 = 0;
                    y2 = image_h;
                }else{
                    double angle_rad= angle * DEG2RAD;
                    x1 = 0;
                    y1 = (double)(r-h/2 - (x1-x_center)* cos(angle_rad) )/sin(angle_rad) + y_center;
                    x2 = image_w;
                    y2 = (double)(r-h/2 - (x2-x_center)*cos(angle_rad) )/sin(angle_rad) + y_center;
                }
                cv::Point start(x1,y1);
                cv::Point end(x2,y2);
                lines.push_back( std::pair<cv::Point,cv::Point>(start,end));
            }
        }
    }
    return lines;
}

/**
 * 获取circle经过hough变换的参数空间
 * 参数分别为r,x,y(三维)
 */ 
cv::Mat Hough::getCirclesSpace(){
    int image_w = image.cols;
    int image_h = image.rows;

    int max_r = std::min(image_w,image_h)/2;
    int min_r = MINR;     //匹配的圆的最小半径, 若存在小于该半径的圆,则会被忽略
    int r_range = max_r - min_r;

    int sizes[] = {image_w,image_h,r_range};

    cv::Mat accu(3,sizes,CV_32S,cv::Scalar::all(0));


    // 为了减少运行时间,提前将sin和cos算好
    //没错, 我就是这么精打细算,哈哈哈哈
    const int step = 6;
    const int divide = 360/step;
    double sins[divide];
    double coss[divide];
    for(int i=0;i<divide;i++)
    {
        double angle_rad = i*step*DEG2RAD;
        sins[i] = sin(angle_rad);
        coss[i] = cos(angle_rad);
    }

    for (int y = 0; y < image_h; y++)
    {
        // std::cout<<"y:"<<y<<std::endl;
        for (int x = 0; x < image_w; x++)
        {
            if (image.data[y * image_w + x] > 250)
            {
                /**
                 * x = a+r*cosθ
                 * y = b+r*sinθ
                 * (a,b)为圆心坐标
                 */
                for(int r=0;r<r_range;r+=2){

                    /**
                     * 没错,这也是为了提高性能
                     * 提前将下面循环中要用的值算好
                     */
                    double x_offset[divide];
                    double y_offset[divide];
                    int R = r+min_r;
                    for (int i = 0; i <divide; i++)
                    {
                        x_offset[i] = R*coss[i];
                        y_offset[i] = R*sins[i];
                    }
                    for(int i = 0;i<divide;i++)
                    {
                        // double angle_rad = angle*DEG2RAD;
                        int a = x - x_offset[i];
                        int b = y - y_offset[i];

                        //落在图像外的圆心不予考虑
                        if(a >= 0 && a<image_w && b>=0 &&b<image_h){
                            long index = (long)a*accu.step[0]+(long)b*accu.step[1]+r*accu.step[2]; 
                            accu.data[index]++;
                        }
                    }
                }
            }
        }
    }
    return accu;
}

/**
 * 获取匹配到的圆
 * @return 所有圆的圆心和半径
 */
std::vector<std::pair<cv::Point,int> > Hough::getCircles(){
    int image_w = image.cols;
    int image_h = image.rows;

    int max_r = std::min(image_w,image_h)/2;
    int min_r = MINR;     //匹配的圆的最小半径, 若存在小于该半径的圆,则会被忽略
    int r_range = max_r - min_r;

    std::cout<<"start"<<std::endl;
    accu = this->getCirclesSpace();
    std::cout<<"end"<<std::endl;
    cv::Size size = accu.size();

    std::vector<std::pair<cv::Point,int> > circles;

    for(int r = 0;r<r_range;r+=2){
        for(int x = 0;x<image_w;x++){
            for(int y=0;y<image_h;y++){
                long index = (long)x*accu.step[0]+(long)y*accu.step[1]+r*accu.step[2]; 
                //0.12 0.4*(r+MINR)
                if(accu.data[index] > 24){
                    bool flag = false;
                    /**
                     * 因为在参数空间中, 投票数较多的点往往是扎堆的
                     * 为了避免最后识别的圆都是密集在一起的
                     * 所以只取局域2MASK*2MASK范围内最大的点为有效点
                     * 以下是两种区域筛选的实现
                     * 方式一基于区域比较
                     * 方式二基于结果比较
                     * 代码只使用了方式一
                     */ 
                    if(true){   //方式一
                        for(int i = -MASK;i<=MASK;i++)
                        {
                            for(int j=-MASK;j<=MASK;j++){
                                //确保检测值不越界
                                if(x+i>=0 && x+i<image_w && y+j>=0 && y+j<image_h){
                                    long index1 = (long)(x+i)*accu.step[0]+(long)(y+j)*accu.step[1]+r*accu.step[2]; 
                                    if( (int)accu.data[index1] > (int)accu.data[index]   || (accu.data[index1] == accu.data[index] && (i<0||j<0)) )
                                    {
                                        flag = true;
                                        i = j = 7; //找到周围点比当前点票数多, 直接跳出循环
                                    }
                                }
                            } 
                        }
                        // 找到周围点比当前点票数多, 直接跳出循环
                        if(flag) continue;
                        std::cout<<cv::Point(x,y)<<":"<<r+MINR<<":"<<(int)accu.data[index]<<std::endl;
                        circles.push_back(std::pair<cv::Point,int>(cv::Point(x,y),r+MINR));
                    }else{  //方式二
                        std::vector<std::pair<cv::Point,int> >::iterator i;
                        bool flag = true; 
                        for(i = circles.begin();i != circles.end();i++){
                            if(x-i->first.x <= MASK && y-i->first.y <= MASK && r - (i->second-MINR) <=MASK){
                                long index1 = (long)(i->first.x)*accu.step[0]+(long)(i->first.y)*accu.step[1]+(i->second-MINR)*accu.step[2]; 
                                //当前点比以前添加的点更好, 则更新结果
                                if(accu.data[index] > (int)accu.data[index1]){
                                    i->first.x = x;
                                    i->first.y = y;
                                    i->second = r+MINR;
                                }
                                flag = false;
                                break;
                            }
                        }
                        if(flag) circles.push_back(std::pair<cv::Point,int>(cv::Point(x,y),r+MINR));
                    }  
                }
            }
        }
    }
    return circles;
}

/**
 * 获取圆的参数空间图像
 */
cv::Mat Hough::getCirclesSpaceImage()
{
    int image_w = image.cols;
    int image_h = image.rows;

    int max_r = std::min(image_w,image_h)/2;
    int min_r = MINR;     //匹配的圆的最小半径, 若存在小于该半径的圆,则会被忽略
    int r_range = max_r - min_r;

    cv::Mat result(image_h,image_w,CV_8UC1,cv::Scalar(0));
    for(int x = 0;x<image_w;x++){
        for(int y=0;y<image_h;y++){
            for(int r = 0;r<r_range;r++){
                long index = (long)x*accu.step[0]+(long)y*accu.step[1]+r*accu.step[2]; 
                if(result.data[y*image_w+x] + accu.data[index] > 255){
                    result.data[y*image_w+x] = 255;
                    break;
                }
                    result.data[y*image_w+x] += accu.data[index];
            }
        }
    }
    return result;
}