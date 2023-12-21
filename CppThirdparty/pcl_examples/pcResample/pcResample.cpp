#include <iostream>
#include <fstream>
#include <cstring>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace std;

int main(int argc, char **argv)
{
	// 定义点云使用的格式：这里用的是XYZRGB
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    // 导入点云
    ifstream pc;
    string filename=argv[1];
    pc.open(filename.c_str());
    //一直读取,直到文件结束
    while(!pc.eof())
    {
        string s;
        //读取一行的内容到字符串s中
        getline(pc,s);
        //如果不是空行就可以分析数据了
        if(!s.empty())
        {
        	// txt点云格式：X Y Z R G B
        	PointT p;
        	double data;
        	//字符串流
        	stringstream ss;
            ss << s;
            // X
            ss >> data;
            p.x = data;
            // Y
            ss >> data;
            p.y = data;
            // Z
            ss >> data;
            p.z = data;
            // R
            ss >> data;
            p.r = data;
            // G
            ss >> data;
            p.g = data;
            // B
            ss >> data;
            p.b = data;
            pointCloud->points.push_back(p);
        }
    }
    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;
    
    // 统计滤波
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    // 统计时考虑查询点临近点数
    statistical_filter.setMeanK(20);
    // 判断是否为离群点的阀值，1个标准差以上就是离群点
    statistical_filter.setStddevMulThresh(1.0);
    statistical_filter.setInputCloud(pointCloud);
    PointCloud::Ptr tmp(new PointCloud);
    statistical_filter.filter(*tmp);
    tmp->swap(*pointCloud);
    cout << "统计滤波之后，点云共有" << pointCloud->size() << "个点." << endl;

    // 体素滤波
    pcl::VoxelGrid<PointT> voxel_filter;
    // 体素边长
    double resolution = 0.04;
    voxel_filter.setLeafSize(resolution, resolution, resolution);
    voxel_filter.setInputCloud(pointCloud);
    voxel_filter.filter(*tmp);
    tmp->swap(*pointCloud);
    cout << "体素滤波之后，点云共有" << pointCloud->size() << "个点." << endl;

	// 保存点云
    pcl::io::savePCDFileBinary("../Statistical_and_voxel_filter.pcd", *pointCloud);
    
    return 0;
}
