/**
  * 具有地标的sfm示例
  * - 地标形成一个 10 米的立方体
  * - 机器人围绕地标旋转，始终面向立方体
  */
/*由于这是一个完整的 3D 问题，我们将使用 Pose3 变量来表示相机位置，
并使用 Point3 变量 (x, y, z) 来表示地标坐标 */
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
// 我们还需要一个相机对象来保存校准信息并执行投影
#include <gtsam/geometry/SimpleCamera.h>

/* ************************************************************************* */
// 创建虚拟路标点真值（正方体的八个顶点）
std::vector<gtsam::Point3> createPoints() 
{
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  return points;
}

/* ************************************************************************* */
// 创建虚拟位姿（路标周围半径30均匀分布的八个相机位置）
std::vector<gtsam::Pose3> createPoses() 
{
  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;
  double radius = 30.0;
  int i = 0;
  double theta = 0.0;
  gtsam::Point3 up(0,0,1);
  gtsam::Point3 target(0,0,0);
  for(; i < 8; ++i, theta += 2*M_PI/8) {
    gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
    //相机面向target点，平移up
    gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
    poses.push_back(camera.pose());
  }
  return poses;
}
/* ************************************************************************* */
