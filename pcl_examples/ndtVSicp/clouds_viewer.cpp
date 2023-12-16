#include <iostream>
#include <thread>
#include <cstring>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

// 查看点云序列的
int main()
{
	pcl::visualization::CloudViewer viewer("viewer");
	pcl::PointCloud<pcl::PointXYZ>::Ptr PointClouds(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 200; i < 800; i++)
	{
		std::string filename = "/home/zard/Videos/" + std::to_string(i) + "target_cloud.pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *PointClouds) == -1)
		{
			PCL_ERROR("Couldn't read file pcd \n");
			return (-1);
		}
		std::cout << "Loaded " << PointClouds->size() << " data points from " << filename << std::endl;
		std::this_thread::sleep_for(100ms);
	}

	return 0;
}
