#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int value=100;
void callback(int,void*);
void mouse(int event,int x,int y,int flags,void*);
Mat img1,img2,imgPoint;

int main()
{
    //设置滑动条：
    img1=imread("1.png");
    namedWindow("滑动条改变图像亮度");
    imshow("滑动条改变图像亮度",img1);
    createTrackbar("亮度百分比","滑动条改变图像亮度",&value,600,callback,0);
    waitKey();
    
    //鼠标响应：
    imshow("鼠标响应 1",img1);
    img1.copyTo(imgPoint);
    imshow("鼠标响应 2",imgPoint);
    
    
    return 0;
}

void callback(int, void*)
{
    float a=value/100.0;
    img2=img1*a;
    imshow("滑动条改变图像亮度",img2);
}

void mouse(int event,int x,int y,int flags,void*)
{
    
}

