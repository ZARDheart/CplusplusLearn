#include <iostream>
#include <thread>
#include <ctime> //时间库，计时用

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// NDT的头文件 NDT使用过程与ICP基本一致，但对调参和场景有要求
#include <pcl/registration/ndt.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

// 控制可视化迭代用的
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

    // --2 导入第二帧新角度的数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../room_scan2.pcd", *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;


	
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
	

	// --4 设置使用机器人里程计找到的初始配准估计值
	// Set initial alignment estimate found using robot odometry.
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();


	// --5 创建并初始化Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	// 设置NDT迭代收敛阈值，即迭代增量的大小，相当于牛顿法中的delata_x，阈值越小，迭代次数越多
	ndt.setTransformationEpsilon(0.0001);
	// Setting maximum step size for More-Thuente line search.
	// 由于NDT算法使用体素化数据结构和More-Thuente线搜索求解位姿增量，因此需要缩放一些参数以适合数据集。
	// 上述参数似乎在我们正在使用的尺度（房间大小）上工作得很好，但如果要处理更小的对象（如咖啡杯的扫描），则需要显著降低这些参数。
	// 这里设置搜索的步长
	ndt.setStepSize(0.1);
	// 设置NDT网格的分辨率：Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution(1.0);
	// 设置最大迭代次数，在大多数情况下，优化器将在达到此限制之前在Transformation Epsilon上终止，但这有助于防止它在错误的方向上运行太长时间。
	ndt.setMaximumIterations(35);
	// Setting point cloud to be aligned.
	// 输入云是将被变换的云，而目标云是输入云将与之对齐的参考系。
	// Target点云按照输入的分辨率划分到3d网格中，并计算每个网格的均值与协方差
	ndt.setInputSource(filtered_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget(target_cloud);

	// --6 准备好对齐点云，所得到的变换后的输入云被存储在输出云中
	// Calculating required rigid transform to align the input cloud to the target cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 配准
	clock_t start = clock(); //获得当前时间
	// guess是align方法的可选参数，不设置时guess默认为单位阵
	// 虽然算法运行并不需要这样的一个初始变换矩阵，但是有了它易于得到更好的结果，尤其是当参考坐标系之间有较大差异时（本例即是）
	ndt.align(*output_cloud, init_guess);
	//ndt.align(*output_cloud);
	double pt = ((double)(clock() - start)) / CLOCKS_PER_SEC; //当前时间-开始标记时间，转化为秒
	cout << "NDT配准用时：" << pt << "s" << endl;
	// 显示对齐的结果以及欧几里得适应度得分，该得分计算为从输出云到目标云中最近点的平方距离之和
	// pcl中的score定义是两份点云之间的误差（error），score越小表示匹配的越准；
	// ndt论文对score的定义是变换后点云的在相应网格的概率，概率越大表示越准，score越大
	// converged代表是否收敛
	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
			  << " score: " << ndt.getFitnessScore() << std::endl;
	cout << "matrix:\n" << ndt.getFinalTransformation() << endl;
	// Transforming unfiltered, input cloud using found transform.
	// 因为我们向算法传递了过滤点云，而不是原始输入云，为了获得原始云的对齐版本，从NDT算法中提取最终变换，并变换原始输入云
	pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
	// Saving transformed input cloud.
	pcl::io::savePCDFileASCII("../room_scan2_transformed_NDT.pcd", *output_cloud);

	/*注意：直接运行例子结果并没有配准？？？
	虽然可以根据匹配初值以及Target网格生成的相关信息（坐标原点，分辨率）直接计算出Source里的每个点落在哪个网格里，
	但是算法实现时仍然采用了kdtree索引的方式搜索每个Source点与Target网格（即网格中心）的关系。
	当初始误差大于网格分辨率时，同样可能会导致算法无法收敛到最优解附近，这也是导致官网例程没有收敛的原因之一
	可以试下把初值的平移误差减小，设置为：（1.79387, 0, 0）*/

	// NDT匹配关系的建立也是通过建立kdtree实现的，而没有使用创建体素网格的索引直接计算得到，
	// 因此实际使用中，如果要使用NDT做实时的定位匹配，需要控制kdtree搜索的次数以减小计算量

	// --7 设置可视化器Initializing point cloud visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer(NDT)"));
	// 背景颜色
	viewer_final->setBackgroundColor(0, 0, 0);
	// 着色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "target cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "output cloud");
	// Starting visualizer
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
