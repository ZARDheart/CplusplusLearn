#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
using namespace cv;

int main()
{
    // --1 Shi-Tomas角点(KeyPoints)检测
    Mat img1=imread("../../opencv/data/1.png"),img2,gray;
    img1.copyTo(img2);        //拷贝原图
    cvtColor(img1,gray,COLOR_BGR2GRAY);//转化为灰度
    int maxCorners=100;       //检测角点最大数量h
    double quality_level=0.01;//质量等级（阈值与最佳角点的比例关系）
    double minDistence=10;    //角点间的最小欧式距离（像素）
    vector<Point2f> corners;  //角点坐标向量
    goodFeaturesToTrack(gray,corners,maxCorners,quality_level,minDistence,Mat(),3,false,0.04);//(1)Input, CV_8UC1 or CV_32FC1.(2)Output vector of corners.(3)Keep this many corners(多了选出最大要求数).(4)(fraction) rel to best(阈值/最佳角点)。(5)Discard corner this close.(6)Ignore corners where mask=0(检测范围蒙板).(7)梯度协方差阵大小（默认3）.(8)false=使用Shi-Tomasi metric(默认),ture=Used Harris metric.(9)常值权重系数（默认0.04）
    //绘制角点（关键点）
    vector<KeyPoint> KeyPoints;
    for(int i=0;i<corners.size();i++)
    {
        KeyPoint k;
        k.pt=corners[i];
        KeyPoints.push_back(k);
    }
    drawKeypoints(img2,KeyPoints,img2,Scalar::all(-1), DrawMatchesFlags::DEFAULT);//原图/关键点向量/输出图像/颜色（默认Scalar::all(-1)）/模式（默认DEFAULT）
    imshow("Shi-Tomas角点检测",img2);
    waitKey();
    imwrite("../../opencv/data/feature/demo6_Shi-Tomas角点检测.png",img2);
    
    // --2 特征点（Features）检测
    //创建ORB特征点类变量
    Ptr<ORB> fea_orb=ORB::create(500, //特征点数目(daf-500)
        1.2f,                         //金字塔层级之间缩放比例(daf-1.2f)
        8,                            //金字塔层数(daf-8)
        31,                           //检测边缘阈值(daf-31)
        0,                            //原图在金字塔的层数(daf-0)
        2,                            //生产单个描述子所用像素点个数(daf-2)
        ORB::FAST_SCORE,              //关键点评价方法(daf-ORB::HARRIS_SCORE)
        30,                           //计算描述子领域尺寸(daf-31)
        20);                          //FAST角点像素差值阈值百分比(daf-20)
    vector<KeyPoint> KPs;
    fea_orb->detect(img1,KPs);//计算关键点:检测图像/关键点/掩膜（默认全图）
    Mat descriptions;
    fea_orb->compute(img1,KPs,descriptions);//计算描述子：检测图像/关键点/描述子
    //fea_orb->detectAndCompute(img1,img1,KPs,descriptions,false);//两步集成：检测图像/掩膜/关键点/描述子/是否使用已有关键点
    //绘制特征点
    Mat img3,img4;
    drawKeypoints(img1,KPs,img3);             //绘制不带角度和大小的特征点
    drawKeypoints(img1,KPs,img4,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);//绘制带角度和大小的特征点
    imshow("不带角度和大小的特征点",img3);
    imshow("带角度和大小的特征点",img4);
    waitKey();
    imwrite("../data/feature/demo6_带角度和大小的FAST特征点.png",img4);
    
    // --3 特征匹配（matching）
    Mat img5=imread("../../opencv/data/2.png"),img6,img7;//img1与img5分别被成为查询/训练描述子集合
    vector<KeyPoint> KPs1;
    Mat descriptions1;
    fea_orb->detectAndCompute(img5,img5,KPs1,descriptions1,false);
    //创建匹配类变量，使用汉明距离：
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //BFMatcher matcher();               //暴力匹配NORM_HANMING
    //FLannBasedMatcher matcher;         //快速最近邻算法匹配（描述子必须CV_32F）
    vector<DMatch> matches;              //匹配信息类变量
    matcher->match(descriptions, descriptions1, matches);//特征匹配：查询/训练描述子，匹配结果/掩膜（默认全图）
    cout<<"匹配数目："<<matches.size()<<endl;
    //绘制未筛选匹配结果
    drawMatches(img1,KPs,img5,KPs1,matches,img6);//左图/左图关键点/右图/右图关键点/匹配结果/输出图像/关键点匹配颜色/未匹配关键点颜色/掩膜
    imshow("未筛选特征点匹配",img6);
    //筛选匹配结果（汉明距离）
    double Maxd=0,Mind=1000;             //匹配的最大最小汉明距离
    for(int i=0;i<matches.size();i++)
    {
        double distence=matches[i].distance;
        if(distence<Mind) Mind=distence;
        if(distence>Maxd) Maxd=distence;
    }
    cout<<"Max Hamming distence :"<<Maxd<<endl<<"Min Hamming distence :"<<Mind<<endl;
    vector<DMatch> good_matches;
    for(int i=0;i<matches.size();i++)
    {
        if(matches[i].distance <= max(2*Mind,30.0))
            good_matches.push_back(matches[i]);
    }
    cout<<"剩余匹配数目："<<good_matches.size()<<endl;
    drawMatches(img1,KPs,img5,KPs1,good_matches,img7);
    imshow("筛选特征点匹配",img7);
    waitKey();
    imwrite("../../opencv/data/feature/demo6_筛选特征点匹配.png",img7);
    
    return 0;
}
