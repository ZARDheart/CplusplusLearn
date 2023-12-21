#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main()
{
    Mat im = Mat::zeros(512,1024,CV_8UC3);
    /*cv::clipLine();判断一条直线是否在给定的矩形内
    cv::ellipse2Po1y();计算一个近似椭圆的多边形
    cv: :fil1ConvexPo1y();画一个填充的简单多边形
    cv: :polyLines();画多重折线*/
    line(im,Point(160,60),Point(370,50),Scalar(255,255,255),2);//画一个简单直线,参数:图像/端点/颜色/线宽
    circle(im,Point(44,50),40,Scalar(0,0,255),-1);             //画一个简单圆：图像/圆心/半径/颜色/线宽（-1实心）
    circle(im,Point(160,60),40,Scalar(0,255,0),3);             //空心，线宽向外扩
    ellipse(im,Point(260,50),Size(40,20),30,0,360,Scalar(0,0,255),-1);//画一个椭圆(可以倾斜):图像/中心/主半轴长/圆弧起/终角度/旋转角度（度）/颜色/线宽（-1实心）
    ellipse(im,Point(370,50),Size(50,20),0,0,180,Scalar(0,255,255),2);//画一个椭圆(只有部分圆弧):图像/中心/主半轴长/旋转角度（度）/颜色/线宽
    rectangle(im,Point(460,10),Point(600,100),Scalar(0,255,255),2);//画一个简单矩形:图像/对角点/颜色/线宽
    rectangle(im,Rect(620,10,140,90),Scalar(0,255,255),-1); //Rect,左上角定点，长和宽
    Point pp1[5]; //第一个多边形的顶点
    pp1[0]=Point(800,10);
    pp1[1]=Point(850,20);
    pp1[2]=Point(930,60);
    pp1[3]=Point(810,80);
    pp1[4]=Point(800,15);
    Point pp2[4];
    pp2[0]=Point(800,110);
    pp2[1]=Point(900,120);
    pp2[2]=Point(960,160);
    pp2[3]=Point(810,180);
    const Point* pts[2]={pp1,pp2}; //要绘制的多边形
    int npts[2]={5,4};  //多边形顶点的个数分别为
    fillPoly(im,pts,npts,2,Scalar(255,255,255),8); //绘制2个多边形
    
    putText(im,"OpenCV4!",Point(50,200),2,1,Scalar(255,255,255));//文本：图像/文本/左下角/字体类型/字体大小/颜色
    imshow("绘图示例",im);
    waitKey(0);
    
    return 0;
}
