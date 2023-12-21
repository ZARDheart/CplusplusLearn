#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main()
{
	// --1 导入第一帧数据(参考的目标)
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../room_scan1.pcd", *target_cloud) == -1)
	{
		PCL_ERROR("Couldn't read file room_scan1.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../room_scan2.pcd", *input_cloud) == -1)
	{
		PCL_ERROR("Couldn't read file room_scan2.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

	// --2 直接合成可视化
	pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer(Direct synthesis)"));
	viewer_final->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(input_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(input_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
	viewer_final->addCoordinateSystem(1.0, "global");
	viewer_final->initCameraParameters();
	// Wait until visualizer window is closed.
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}

    return 0;
}
