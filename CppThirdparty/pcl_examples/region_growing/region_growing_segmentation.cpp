#include <vtkAutoInit.h>
// VTK_MODULE_INIT(vtkRenderingOpenGL)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>//鍖哄煙鐢熼暱鍒嗗壊
#include <pcl/features/normal_3d.h>//娉曠嚎鐗瑰緛浼拌
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
 
using namespace pcl;
using namespace std;
typedef PointXYZ PoinT;
int *rand_rgb() {//闅忔満浜х敓棰滆壊
	int *rgb = new int[3];
	rgb[0] = rand() % 255;
	rgb[1] = rand() % 255;
	rgb[2] = rand() % 255;
	return rgb;
}
int main(int argc, char **argv) {
	PointCloud<PoinT>::Ptr cloud(new PointCloud<PoinT>);
	if (io::loadPCDFile(argv[1], *cloud) == -1) {
		PCL_ERROR("read false");
	}
	//闄嶉噰鏍�
	VoxelGrid<PoinT> vox;
	PointCloud<PoinT>::Ptr vox_cloud(new PointCloud<PoinT>);
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.25, 0.25, 0.25);
	vox.filter(*vox_cloud);//鎵ц婊ゆ尝锛屽苟灏嗙粨鏋滀繚瀛樺湪vox_cloud涓�
	//鍘诲櫔澹�
	StatisticalOutlierRemoval<PoinT>sor;
	PointCloud<PoinT>::Ptr sor_cloud(new PointCloud<PoinT>);
	sor.setInputCloud(vox_cloud);
	sor.setMeanK(20);//璁剧疆鑰冭檻鏌ヨ鐐逛复杩戜釜鏁颁负20
	sor.setStddevMulThresh(0.05);//璁剧疆鍒ゆ柇鏄惁涓虹缇ょ偣鐨勯槇鍊间负0.02
	sor.filter(*sor_cloud);//鎵ц婊ゆ尝锛屽苟灏嗙粨鏋滀繚瀛樺湪sor_cloud涓�
	//娉曞悜閲忔眰瑙�
	NormalEstimation<PoinT, Normal> ne;//鍒涘缓娉曞悜閲忓璞�
	search::KdTree<PoinT>::Ptr tree(new search::KdTree<PoinT>);
	PointCloud<Normal>::Ptr normal_cloud(new PointCloud<Normal>);
	ne.setInputCloud(sor_cloud);//璁剧疆杈撳叆鐐逛簯
	ne.setKSearch(20);//璁剧疆kdtree鎼滅储璺濈
	ne.setSearchMethod(tree);//璁剧疆鎼滅储鏂瑰紡
	ne.compute(*normal_cloud);//杩涜娉曞悜閲忔眰瑙�
	//鍩轰簬娉曞悜閲忓拰鏇茬巼鐨勫尯鍩熺敓闀跨畻娉�
	PointCloud<PoinT>::Ptr reg_cloud(new PointCloud<PoinT>);//鍒涘缓reg_cloud鎸囬拡
	RegionGrowing<PoinT, Normal> reg;//鍒涘缓鍖哄煙鐢熼暱绠楁硶鍒嗗壊瀵硅薄
	reg.setInputCloud(sor_cloud);//璁剧疆杈撳叆鐐逛簯
	reg.setSearchMethod(tree);//璁剧疆鎼滅储鏂瑰紡
	reg.setNumberOfNeighbours(20);//璁剧疆鎵€鎼滈偦鍩熺偣鐨勪釜鏁�
	reg.setMinClusterSize(50);//鏈€灏忚仛绫荤殑鐐规暟
	//reg.setMaxClusterSize(100000);//鏈€澶ц仛绫荤殑鐐规暟
	reg.setSmoothnessThreshold(6.0 / 180 * M_PI);//璁剧疆骞虫粦搴� 娉曠嚎鎻掑€奸槇鍊� 3搴�
	reg.setCurvatureThreshold(1.0);//璁剧疆鏇茬巼鐨勯槇鍊�
	reg.setInputNormals(normal_cloud);//杈撳叆鐨勬硶绾�
 
	vector<PointIndices>clusters;
	reg.extract(clusters);
	ExtractIndices<PoinT>ext;
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("Result of RegionGrowing"));
	for (int iter = 0; iter < clusters.size(); iter++)
	{
		//璋冪敤鏂瑰紡 灏嗙储寮曟眹鎬�
		PointCloud<PoinT>::Ptr final_cloud(new PointCloud<PoinT>);
		vector<int> inlier = clusters[iter].indices;
		//鎻愬彇姣忎釜绱㈠紩瀵瑰簲寰楃偣浜戝苟灞曠ず
		boost::shared_ptr<vector<int>>index = boost::make_shared<vector<int>>(inlier);
		ext.setInputCloud(sor_cloud);
		ext.setIndices(index);
		ext.setNegative(false);
		ext.filter(*final_cloud);//鎵ц绱㈠紩锛屽苟灏嗙粨鏋滀繚瀛樺湪final_cloud涓�
 
		stringstream ss;
		ss << "/home/zard/Music/region_growing/RegionGrowing_clouds/" << "RegionGrowing_clouds" << iter << ".pcd";
		io::savePCDFileASCII(ss.str(), *final_cloud);
		int *rgb = rand_rgb();//闅忔満鐢熸垚0-255鐨勯鑹插€�
		visualization::PointCloudColorHandlerCustom<PoinT>rgb1(final_cloud, rgb[0], rgb[1], rgb[2]);//鎻愬彇鐨勫钩闈笉鍚屽僵鑹插睍绀�
		delete[]rgb;
		viewer->addPointCloud(final_cloud, rgb1, ss.str());
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());//璁剧疆鐐圭殑澶у皬
	}
	viewer->spin();
 
	return 0;
}
