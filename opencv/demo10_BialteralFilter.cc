#include<opencv/cv.h>
#include<opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d/features2d.hpp>
#include<opencv2/opencv.hpp>                     // 双边高斯滤波的头文件。 
#include<iostream>
#include<algorithm>
using namespace cv;
using namespace std;

//种子点
struct SeedPoint
{
    cv::Point2i l_pt;
    cv::Point2i r_pt;
    double diff;
};

class Reconstructor
{
public:
    Reconstructor(cv::Mat &Depth);
    //当前关键帧对应位姿，图像以及畸变参数
    cv::Mat mImDepth,mImLeft, mImRight;
    ///双边滤波边长
    int N,size;
    ///双边滤波空间域标准差，灰度域标准差
    float sigmas,sigmar;
    ///双边滤波空间域权值(二位数组)
    float **spaceArray;
    ///双边滤波灰度域权值(一位数组)，包含的是所有灰度差的情况，灰度差作为数组索引，获取对应权值
    float *colorArray;
    double mbf;
    //种子特征点
    vector<cv::KeyPoint> mvKeys;
    vector<double> mvuRight;
    
    ///图像扩充图像满足窗口计算（扩展N）
    void ExternIm(cv::Mat &m);
    ///反扩充
    void ArcExternIm(cv::Mat &m);
    ///深度图像双边滤波
    void BialteralFilter();
    ///计算匹配代价
    pair<float,float> SAD(const cv::Point2i &L,const cv::Point2i &R);
    ///计算双目深度图
    void ComputStereoDepth();
};

Reconstructor::Reconstructor(cv::Mat &Depth):mImDepth(Depth),N(20),sigmas(6),sigmar(0.3),mbf(386.1448)
{
        //计算空间域权值
        size=2*N+1;
        // 初始化数组
        float **_spaceArray = new float*[size ];   //多一行，最后一行的第一个数据放总值
        for (int i = 0; i < size; i++) {
            _spaceArray[i] = new float[size];
        }
        // 高斯分布计算
        int ci=N, cj=N;
        for (int i = 0; i < size; i++){
            for (int j = 0; j < size; j++){
                _spaceArray[i][j] =
                exp(-(1.0f)* (((i - ci)*(i - ci) + (j - cj)*(j - cj)) /(2.0f*sigmas*sigmas)));
            }
        }spaceArray=_spaceArray;
        
        //计算灰度域权值
        float*_colorArray = new float[255];
        for (int n = 0; n < 255; n++) {
            _colorArray[n] = exp((-1.0f*(n*n)) / (2.0f*sigmar*sigmar));
        }
        colorArray=_colorArray;
}

void Reconstructor::BialteralFilter()
{
        cv::Mat dst= mImDepth.clone();
        for (int i = 0; i < mImDepth.rows; i++)
        {
            const float *current = mImDepth.ptr<float>(i);
            for (int j = 0; j < mImDepth.cols; j++)
            {
                if (i < N || j < N || mImDepth.rows - i <= N || mImDepth.cols - j <= N)
                    continue;
                float weight_quan = 0, eve_weigth = 0;
                for (int k = -N; k <= N; k++)
                {
                    const float *current1 = mImDepth.ptr<float>(i+k);
                    for (int l = -N; l <= N; l++)
                    {
                        //灰度差
                        float Color_dif = abs(current[j] - current1[j + l]);
                        //每一个通道点值的符合权重
                        float recom_weigth = spaceArray[k + N][l + N] * colorArray[int(Color_dif )];
                        eve_weigth += mImDepth.ptr<float>(i+k)[j+l] * recom_weigth;
                        weight_quan += recom_weigth;
                    }
                }
                dst.ptr<float>(i)[j] = eve_weigth / weight_quan;
            }
        }
        mImDepth= dst.clone();
}

void Reconstructor::ExternIm(cv::Mat &m)
{
        if(m.type()!=CV_32F)
        {
            m.convertTo(m,CV_32F);
        }
        cv::Mat mN(m.rows+2*N,m.cols+2*N,CV_32F);
        for(int i=0;i<mN.rows;i++)
        {
            for(int j=0;j<mN.cols;j++)
            {
                if(i<N && j<N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(0)[0];
                else if(i<N && j>=N && j<mN.cols-N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(0)[j-N];
                else if(i<N && j>=mN.cols-N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(0)[m.cols-1];
                
                else if(i>=N &&i<mN.rows-N && j<N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(i-N)[0];
                else if(i>=N &&i<mN.rows-N && j>=mN.cols-N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(i-N)[m.cols-1];
                
                else if(i>=mN.rows-N && j<N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(m.rows-1)[0];
                else if(i>=mN.rows-N && j>=N && j<mN.cols-N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(m.rows-1)[j-N];
                else if(i>=mN.rows-N && j>=mN.cols-N)
                    mN.ptr<float>(i)[j]=m.ptr<float>(m.rows-1)[m.cols-1];
                
                else
                    mN.ptr<float>(i)[j]=m.ptr<float>(i-N)[j-N];
            }
        }
        m=mN;
}

void Reconstructor::ArcExternIm(cv::Mat& m)
{
        cv::Mat mA(m.rows-2*N,m.cols-2*N,CV_32F);
        for(int i=0;i<mA.rows;i++)
        {
            for(int j=0;j<mA.cols;j++)
            {
                    mA.ptr<float>(i)[j]=m.ptr<float>(i+N)[j+N];
            }
        }
        m=mA;
        m.convertTo(m,CV_8UC1);
}

pair<float,float>  Reconstructor::SAD(const cv::Point2i &L,const cv::Point2i &R)
{
        //SAD窗口半径
        int w=5;
        // 提取左图中，以点(L.x,L.y)为中心, 半径为w的图像块patch
        cv::Mat IL = mImLeft.rowRange(L.y-w,L.y+w+1).colRange(L.x-w,L.x+w+1);
        // 图像块均值归一化，降低亮度变化对相似度计算的影响
        IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);
       cout<<"Here1"<<endl;
        //在右图半径为r的范围内搜索,找到最小SAD与其对应的x坐标
        int r=5,x=0;
        map<int,float> sad;
        vector<float> sa;
        float minsad=MAXFLOAT;
        for(int i=-r;i<=r;i++)
        {
            for(int j=-r;j<=r;j++)
            {
                cv::Mat IR = mImRight.rowRange(R.y-w+i,R.y+w+i+1).colRange(R.x-w+j,R.x+w+j+1);
                IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);
                cout<<"Here2"<<endl;
                float s=cv::norm(IL,IR,cv::NORM_L1);
                sad[R.x+j]=s;
                sa.push_back(s);
                if(s<minsad) 
                {
                    minsad=s;
                    x=R.x+j;
                }
            }
        }
        
        //抛物线插值
        float dist1,dist2,dist3;
        if(sad.count(x-1)==0)
        {
        	dist1 = sad[x];
		    dist2 = sad[x+1];
		    dist3 = sad[x+2];
		}
		else if(sad.count(x+1)==0)
		{
        	dist1 = sad[x-2];
		    dist2 = sad[x-1];
		    dist3 = sad[x];
		}
		else
		{
			dist1 = sad[x-1];
		    dist2 = sad[x];
		    dist3 = sad[x+1];
		}
		    
		float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
		float bestuR =x+deltaR;
		    
		//归一化sad最小，并不代表就一定是匹配的，比如光照变化、弱纹理、无纹理等同样会造成误匹配
		sort(sa.begin(),sa.end());
		float median = sa[int(sa.size()/2)];
		
		map<int,float>::iterator it;
		for(it=sad.begin();it!=sad.end();it++)
		{
			if(it->second>1.5 * 1.4 * median)
				return pair<float,float>(0,-1);
		}
		return pair<float,float>(bestuR,minsad);
}
    
    /* 稠密立体匹配计算双目图像的深度图，步骤
    1.提取特征点并匹配（帧构造的时候已经完成）.
    2.基于特征点区域生长的视差稠密化.
    3.亚像素精度优化.步骤2得到的视差为uchar/int类型，不一定是真实视差，通过亚像素差值(抛物线插值)获取float精度的视差.
    4.删除离群点(outliers).相似度阈值判断，归一化sad最小，并不代表就一定是正确匹配，比如光照变化、弱纹理等会造成误匹配 */
    void Reconstructor::ComputStereoDepth()
    {
        cv::Mat map1(mImLeft.size(), CV_8UC1, cv::Scalar::all(0)); //左匹配图，0表示未匹配, 1表示已匹配
        mImDepth=cv::Mat::zeros(mImLeft.size(),CV_32F);
        list<SeedPoint> seeds;

        for (int i = 0; i < mvuRight.size(); ++i)
        {
            SeedPoint seed;
            seed.l_pt = cv::Point2i(mvKeys[i].pt.x+N,mvKeys[i].pt.y+N);
            seed.r_pt = cv::Point2i(mvuRight[i]+N,mvKeys[i].pt.y+N);
            pair<float,float> sad=SAD(seed.l_pt,seed.r_pt);
            seed.diff = sad.second;

            if(seed.diff>=0)
            {
                seeds.push_back(seed);
                map1.ptr<uchar>(int(seed.l_pt.y))[int(seed.l_pt.x)] = 1;
                mImDepth.ptr<float>(int(seed.l_pt.y))[int(seed.l_pt.x)] = mbf/(mvKeys[i].pt.x-sad.first);
            }
        }
        cout<<"seed number:"<<seeds.size()<<endl;
        
        while (!seeds.empty())
        {
            SeedPoint seed = seeds.front();
            seeds.pop_front();
            
            for (int i = -1; i <= 1; i++){
                for (int j = -1; j <= 1; j++){
                    cv::Point2i p1(seed.l_pt.x + j, seed.l_pt.y + i);
                    cv::Point2i p2(seed.r_pt.x + j, seed.r_pt.y + i);
                    
                    if (p1.y>N && p1.x>N && (p1.y<mImLeft.rows-N) && (p1.x<mImLeft.cols-N) 
                    	&& map1.ptr<uchar>(int(p1.y))[int(p1.x)] == 0)
                    {
                        cout<<seeds.size()<<endl;
                        pair<float,float> sad=SAD(p1,p2);
                        if (sad.second>=0)
                        {
                            SeedPoint new_seed;
                            new_seed.diff = sad.second;
                            new_seed.l_pt = p1;
                            new_seed.r_pt = cv::Point2i(sad.first,p1.y);
                            seeds.push_back(new_seed);
                            mImDepth.ptr<uchar>(int(p1.y))[int(p1.x)] = mbf/(p1.x-sad.first);
                            map1.ptr<uchar>(int(p1.y))[int(p1.x)] = 1;
                        }
                        
                    }
                }
            }
        }
    }
    
int main()
{
        /*//双边滤波
        cv::Mat src = cv::imread("../../opencv/data/depth/test.png");
        cv::Mat dst=src.clone();
        cvtColor(dst,dst,CV_RGB2GRAY);
        Reconstructor R(dst);
        R.ExternIm(R.mImDepth);
        R.BialteralFilter();
        cv::imshow("src", src);
        cv::imshow("BialteralFilter with bar", R.mImDepth);
        R.ArcExternIm(R.mImDepth);
        cv::imshow("BialteralFilter", R.mImDepth);*/
        
        //特征点提取
        cv::Mat src1 = cv::imread("../../opencv/data/L.png");
        cv::Mat src2 = cv::imread("../../opencv/data/R.png");
        Ptr<ORB> fea_orb=ORB::create(500, 1.2f, 8,31, 0,2,  ORB::FAST_SCORE, 30,20); 
        vector<KeyPoint> KPs1,KPs2;
        Mat descriptions1,descriptions2,img,imgk;
        
        fea_orb->detect(src1,KPs1);
        fea_orb->compute(src1,KPs1,descriptions1);
        fea_orb->detect(src2,KPs2);
        fea_orb->compute(src2,KPs2,descriptions2);
        if((descriptions1.type()!=CV_32F) || (descriptions2.type()!=CV_32F))
        {
            descriptions1.convertTo(descriptions1,CV_32F);
            descriptions2.convertTo(descriptions2,CV_32F);
        }
        FlannBasedMatcher matcher;
        vector<DMatch> matches;
        matcher.match(descriptions1, descriptions2, matches);
        /*
        drawKeypoints(src1,KPs1,imgk); 
        drawMatches(src1,KPs1,src2,KPs2,matches,img);
        imshow("features",imgk);
        imshow("matchs",img);
        cv::waitKey(0);*/
        
        //立体匹配
        vector<KeyPoint> K1,K2;
        vector<double> ku;
        for(int i=0;i<matches.size();i++)
        {
            K1.push_back(KPs1[matches[i].queryIdx]);
            K2.push_back(KPs2[matches[i].trainIdx]);
            ku.push_back(KPs2[i].pt.x);
        }
        cv::Mat depth=cv::Mat(src1.size(),CV_32F);
        Reconstructor R1(depth);
        R1.mvKeys=K1;
        R1.mvuRight=ku;
        R1.mImLeft=src1;
        R1.mImRight=src2;
        if(R1.mImLeft.type()==CV_8UC3)
        {
            cvtColor(R1.mImLeft,R1.mImLeft,CV_RGB2GRAY);
            cvtColor(R1.mImRight,R1.mImRight,CV_RGB2GRAY);
        }
        if(R1.mImLeft.type()!=CV_32F || R1.mImLeft.type()==CV_8UC1)
        {
            R1.mImLeft.convertTo(R1.mImLeft,CV_32F);
            R1.mImRight.convertTo(R1.mImRight,CV_32F);
        }
        R1.ExternIm(R1.mImDepth);
        R1.ExternIm(R1.mImLeft);
        R1.ExternIm(R1.mImRight);
        R1.ComputStereoDepth();
        R1.BialteralFilter();
        R1.ArcExternIm(R1.mImDepth);
        cv::imshow("Result", R1.mImDepth);
        
        cv::waitKey(0);
}
