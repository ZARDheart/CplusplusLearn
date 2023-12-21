#include <iostream>
#include <thread>
#include <ctime> //时间库，计时用

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
// NDT的头文件
#include <pcl/registration/ndt.h>

using namespace std::chrono_literals;

int main()
{
	// --1 导入第一帧数据(参考的目标) Loading first scan of room.
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../room_scan1.pcd", *target_cloud) == -1)
	{
		PCL_ERROR("Couldn't read file room_scan1.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    // --2 自己设置一个变换将一帧数据变换到另一坐标系下，然后再配准看看效果（这时已知真值）
	Eigen::AngleAxisf true_rotation(-0.7854, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f true_translation(-1, 1, 0);
	Eigen::Matrix4f true_transf = (true_translation * true_rotation).matrix();
    cout << "true matrix:\n" << true_transf.inverse() << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*target_cloud, *input_cloud, true_transf);

    // --3 滤波到原来数据的10%，加快匹配速度
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);


	// --4 重新设置使用机器人里程计找到的初始配准估计值
	Eigen::AngleAxisf init_rotation(0.6, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1, 0.2, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // --5 创建并初始化Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.0001);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);
	ndt.setInputSource(filtered_cloud);
	ndt.setInputTarget(target_cloud);

    // --6 准备好对齐点云，所得到的变换后的输入云被存储在输出云中
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	clock_t start = clock();
	ndt.align(*output_cloud, init_guess);
	double pt = ((double)(clock() - start)) / CLOCKS_PER_SEC;
	cout << "NDT配准用时：" << pt << "s" << endl;
	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
			  << " score: " << ndt.getFitnessScore() << std::endl;
	cout << "matrix:\n" << ndt.getFinalTransformation() << endl;
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
	pcl::io::savePCDFileASCII("../room_scan3_transformed_NDT.pcd", *output_cloud);

	// --7 设置可视化器Initializing point cloud visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "target cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "output cloud");
	viewer_final->addCoordinateSystem(1.0, "global");
	viewer_final->initCameraParameters();
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}

	return 0;
}