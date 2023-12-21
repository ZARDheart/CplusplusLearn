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

// 定义点云使用的格式：这里用的是XYZI
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv)
{
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
        	// txt点云格式：X Y Z I
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
            // I
            ss >> data;
            p.intensity = data;
            pointCloud->points.push_back(p);
        }
    }
    pointCloud->is_dense = false;
    cout << "读取成功，点云共有" << pointCloud->size() << "个点." << endl;
    
    string file=filename+".pcd";
    // 保存点云
    pcl::io::savePCDFileBinary(file, *pointCloud);
    
    return 0;
}
