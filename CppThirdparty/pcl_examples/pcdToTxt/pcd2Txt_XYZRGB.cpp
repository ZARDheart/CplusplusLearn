#include <iostream>
#include <cstring>
#include <boost/format.hpp>  // for formating strings
#include <sstream>//字符串流支持
#include <fstream> //读写文本
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;

// typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

int main(int argc, char **argv) 
{

    // Load the points
    string filename=argv[1];
    PointCloudPtr cloud(new PointCloud);
    if (argc == 0 || pcl::io::loadPCDFile(filename, *cloud)) 
    {
        cout << "failed to load point cloud!";
        return 1;
    }
    cout << "point cloud loaded, points: " << cloud->points.size() << endl;
    
    //写入数据:
    ofstream testFile;
    string file=filename+"2XYZRGB.txt";
    cout<<file<<endl;
    testFile.open(file); //没有自动创建
    if(testFile.is_open()) //文件是否打开
    {
        testFile<<fixed; //开始写入,与cout相同
        testFile.precision(4); //写入小数精度
        int count = cloud->points.size();
        for(int i=0;i<count;i++)
        {
        	testFile<<cloud->points[i].x<<" "<<cloud->points[i].y <<" "<<cloud->points[i].z
        	<<" "<<std::to_string(cloud->points[i].r)<<" "<<std::to_string(cloud->points[i].g)<<" "<<std::to_string(cloud->points[i].b)<<endl; //写入数据（覆盖）
        }
        testFile.close();
    }
    else
    {
    	cout<<"failed to save point cloud!"<<endl;
    }
}
