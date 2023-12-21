#include <iostream>
#include <thread>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// icp的头文件
#include <pcl/registration/icp.h>
// NDT的头文件
#include <pcl/registration/ndt.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main()
{
	// --1 导入数据
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

	
	// --2 滤波到原来数据的10%，加快匹配速度
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);
	std::cout << "Filtered cloud contains " << filtered_cloud->size()
		<< " data points from room_scan2.pcd" << std::endl;
	

	// --3 设置使用机器人里程计找到的初始配准估计值
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	// --4 创建ndt并初始化
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(40);
	ndt.setInputSource(filtered_cloud);
	ndt.setInputTarget(target_cloud);

	// --5 准备对齐点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 配准
	clock_t start = clock();
    // NDT 配准
	ndt.align(*output_cloud, init_guess);
    // ICP 配准
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(target_cloud);
    icp.setTransformationEpsilon(1e-10);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setEuclideanFitnessEpsilon(0.00005);
    icp.setMaximumIterations(30);
    icp.setUseReciprocalCorrespondences(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    // 用ndt的结果当初值
    icp.align(*cloud_final,ndt.getFinalTransformation());
	double pt = ((double)(clock() - start)) / CLOCKS_PER_SEC;
	cout << "NDT_and_ICP 配准用时：" << pt << "s" << endl;

	std::cout << "Normal Distributions Transform has converged:" << icp.hasConverged()
			  << " score: " << icp.getFitnessScore() << std::endl;
	cout << "matrix:\n" << icp.getFinalTransformation() << endl;
	pcl::transformPointCloud(*input_cloud, *cloud_final, icp.getFinalTransformation());
	pcl::io::savePCDFileASCII("../ICP_and_NDT.pcd", *cloud_final);

	// --6 设置可视化器Initializing point cloud visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer(NDT)"));
	viewer_final->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(cloud_final, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud_final, output_color, "cloud_final");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_final");
	viewer_final->addCoordinateSystem(1.0, "global");
	viewer_final->initCameraParameters();
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}

	return 0;
}