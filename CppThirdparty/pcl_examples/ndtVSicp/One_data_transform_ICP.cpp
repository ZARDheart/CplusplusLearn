#include <iostream>
#include <thread>
#include <ctime> //时间库，计时用
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// icp的头文件
#include <pcl/registration/icp.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

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
	// Filtering input scan to roughly 10% of original size to increase speed of registration.
	// 目标云不需要过滤，因为NDT算法所使用的体素网格数据结构不使用单独的点，而是使用包含在其每个数据结构体素单元中的点的统计数据。
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);
	std::cout << "Filtered cloud contains " << filtered_cloud->size()
		<< " data points from room_scan2.pcd" << std::endl;

    // --4 创建ICP的实例类
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // icp.setInputSource(input_cloud);
    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(target_cloud);
    // 设置两次变化矩阵之间的差值（一般设置为1e-10即可）
    icp.setTransformationEpsilon(1e-10);
    // 添加一个最大距离的阈值，超过最大距离的点不作为对应点。//设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setMaxCorrespondenceDistance(1);
    // 设置收敛条件是均方误差和小于阈值(第二个收敛条件)， 停止迭代；
    // icp.setEuclideanFitnessEpsilon(0.00005);
    // 最大迭代次数，icp是一个迭代的方法，最多迭代这些次（若结合可视化并逐次显示，可将次数设置为1）；
    icp.setMaximumIterations(35);
    // 设置是否使用相互对应关系
    icp.setUseReciprocalCorrespondences(true);


	// --5 重新设置使用机器人里程计找到的初始配准估计值
	Eigen::AngleAxisf init_rotation(0.6, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1, 0.2, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // --6 配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    // icp执行计算，并将变换后的点云保存在cloud_final里
    // icp.align(*cloud_final); //不使用初值
	clock_t start = clock(); //获得当前时间
    icp.align(*cloud_final,init_guess);
	double pt = ((double)(clock() - start)) / CLOCKS_PER_SEC; //当前时间-开始标记时间，转化为秒
	cout << "ICP配准用时：" << pt << "s" << endl;
    // 如果由于设置相关参数过于苛刻，而造成没有匹配成功法，icp.hasConverged() 会提示匹配失败，
    cout << "has conveged:" << icp.hasConverged() << "score:" << icp.getFitnessScore() << endl;
    // 并且  icp.getFitnessScore()  为0，相关的转换矩阵为单位矩阵。
    cout << "matrix:\n" << icp.getFinalTransformation() << endl;
    pcl::transformPointCloud(*input_cloud, *cloud_final, icp.getFinalTransformation());
    // Saving transformed input cloud.
	pcl::io::savePCDFileASCII("../room_scan3_transformed_ICP.pcd", *cloud_final);

    // --7 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "target cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(cloud_final, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud_final, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "output cloud");
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