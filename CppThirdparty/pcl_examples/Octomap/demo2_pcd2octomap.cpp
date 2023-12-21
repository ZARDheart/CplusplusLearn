#include <iostream>
#include <cstring>
#include <boost/format.hpp> // for formating strings
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>

using namespace std;

// typedefs
// typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

int main(int argc, char **argv)
{
    // Load the points
    string filename = argv[1];
    PointCloudPtr cloud(new PointCloud);
    if (argc == 0 || pcl::io::loadPCDFile(filename, *cloud))
    {
        cout << "failed to load point cloud!";
        return 1;
    }
    cout << "point cloud loaded, points: " << cloud->points.size() << endl;

    // octomap tree
    octomap::OcTree tree(0.1);        // 参数为分辨率
    octomap::Pointcloud cloudoctomap; // the point cloud in octomap
    int count = cloud->points.size();
    for (int i = 0; i < count; i++)
    {
        cloudoctomap.push_back(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        cout<<i<<endl;
    }
    tree.insertPointCloud(cloudoctomap, octomap::point3d(0, 0, 0));
    // 将点云存入八叉树地图，给定原点，这样可以计算投射线

    // tree.updateInnerOccupancy();
    tree.writeBinary(filename + "octomap.bt");

    return 0;
}