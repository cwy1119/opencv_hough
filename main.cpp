#include <iostream>
#include <unistd.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "hough.h"

using namespace cv;
using std::cout;
using std::string;

const int MATCH_LINE_CIRCLE = 0;
const int MATCH_LINE = 1;
const int MATCH_CIRCLE = 2;

 string imagePath;
int choice = MATCH_LINE_CIRCLE;

/**
 * 用法提示
*/
void usage(char* s)
{
    cout<<"\n";
    cout<<s<<" -l(/c/a)  <source file>\n";
    cout<<"    l: match the lines only\n";
    cout<<"    c: match the circles only\n";
    cout<<"    a: match the lines and circles\n";
    cout<<"eg: "<<s<<" -l  .\\example.jpg\n";
    cout<<"\n";
}

void doMatch(string filename, int choice)
{
    Mat imgOri;     //原始图
    Mat imgBlur;    //blur提取后的结果
    Mat imgEdge;    //使用canny算子检测出的边缘图
    Mat imgRes;     //在原始图上添加检测形状的结果图
    Mat imgAccu;    //参数空间图
    const string CW_EDGE = "edge image";
    const string CW_ACCUMULATOR = "accumulator space";
    const string CW_RESULT = "result image";

    //在进行形状识别之前, 先对原图进行边缘检测
    imgOri = imread(filename,1);
    blur(imgOri,imgBlur,cv::Size(5,5));
    cv::Canny(imgBlur,imgEdge,100,150,3);
    
    Hough hough;
    hough.setImage(imgEdge);

    imgRes = imgOri.clone();

    //检测直线
    if(choice == MATCH_LINE){
        std::vector<std::pair<cv::Point,cv::Point> > lines = hough.getLines(200);
        std::vector<std::pair<cv::Point,cv::Point> >::iterator i;
        for(i = lines.begin();i != lines.end();i++){
            cv::line(imgRes,i->first,i->second, cv::Scalar( 0, 255, 255), 2, 8);
            cout<<i->first<<":"<<i->second<<std::endl;
        }
        cout<<"lines:"<<lines.size()<<std::endl;
        imgAccu= hough.getLinesSpaceImage();
    }else if(choice == MATCH_CIRCLE){
        std::cout<<"It takes time, wait a moment please.."<<std::endl;
        std::vector<std::pair<cv::Point,int> > circles = hough.getCircles();
        std::vector<std::pair<cv::Point,int> >::iterator i;
        for(i = circles.begin();i != circles.end();i++){
            cv::circle(imgRes,i->first,i->second, cv::Scalar( 0, 255, 255), 2, 8);
            cout<<i->first<<":"<<i->second<<std::endl;
        }
        cout<<"circles:"<<circles.size()<<std::endl;
        imgAccu= hough.getCirclesSpaceImage();
    }
    
    cv::imshow(CW_ACCUMULATOR,imgAccu);
    cv::imshow(CW_RESULT,imgRes);
    cv::imshow(CW_EDGE,imgEdge);
    cv::imwrite("output.jpg",imgRes);
    waitKey();
}


int main(int argc,char** argv)
{
    int c;
    while( ((c = getopt( argc, argv, "l:c:a:?" ) ) ) != -1 )
    {
        switch (c)
        {
        case 'l':
            imagePath   =   optarg;
            choice      =   MATCH_LINE;
            break;
        case 'c':
            imagePath   =   optarg;
            choice      =   MATCH_CIRCLE;
            break;
        case 'a':
            imagePath   =   optarg;
            choice      =   MATCH_LINE_CIRCLE;
        default:
            usage(argv[0]);
            break;
        }
    }

    if(imagePath.empty())
    {
        usage(argv[0]);
        return -1;
    }

    doMatch(imagePath,choice);
}