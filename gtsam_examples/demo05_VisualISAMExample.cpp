/** @brief 模拟数据集上的结构运动问题的视觉 SLAM 示例。此版本使用 iSAM 增量解决问题 */

// For loading the data
#include "SFMdata.h"

// 地标（即像素坐标）的像素坐标将存储为 Point2 (x, y)
#include <gtsam/geometry/Point2.h>

/*系统中的每个变量（姿势和地标）都必须用唯一的键标识，
我们可以使用简单的整数键 (1, 2, 3, ...) 或符号 (X1, X2, L1)，这里我们将使用符号*/
#include <gtsam/inference/Symbol.h>

// 在 GTSAM 中，测量函数表示为“因子”。 几个共同因子已提供用于解决机器人/SLAM/Bundle Adjustment 问题的库。
// 这里我们将使用投影因子来模拟相机的地标观测。此外，我们将使用先验因子在某个位置初始化机器人。
#include <gtsam/slam/PriorFactor.h>  // 一元因子，系统先验
#include <gtsam/slam/ProjectionFactor.h>

// 我们想使用 iSAM 逐步解决 structure-from-motion 问题，所以在此处包含 iSAM
#include <gtsam/nonlinear/NonlinearISAM.h>

// iSAM 要求将一组新的因子作为输入更新自身，存储在因子图中，以及添加因子中使用的新变量的初始估计
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>
using namespace std;
using namespace gtsam;


int main(int argc, char *argv[])
{
  // 相机标定参数:fx_(1), fy_(1), s_(0), u0_(0), v0_(0)，s为坐标轴倾斜参数，理想情况下为0
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // 定义像素噪声模型（二维一个像素）
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();
  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();

  // 创建一个 NonlinearISAM 对象，它将在每次“relinearizeInterval”更新时对变量进行重新线性化和重新排序
  int relinearizeInterval = 3;
  NonlinearISAM isam(relinearizeInterval);

  // 创建一个因子图和初始值来保存新的数据，用于更新iSAM
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // 循环遍历不同的姿态，逐步将测量添加到 iSAM
  for (size_t i = 0; i < poses.size(); ++i)
  {
    // 为每个路标点投影添加因子
    for (size_t j = 0; j < points.size(); ++j)
    {
      // Create ground truth measurement
      SimpleCamera camera(poses[i], *K);
      // 投影点
      Point2 measurement = camera.project(points[j]);
      // 添加重投影因子：
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(measurement, noise, Symbol('x', i), Symbol('l', j), K);
    }

    // 从真实值添加噪声，初始化变量
    Pose3 noise(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
    Pose3 initial_xi = poses[i].compose(noise);
    // 为当前位姿添加一个初始估计
    initialEstimate.insert(Symbol('x', i), initial_xi);

    // 如果这是第一次迭代，则在第一个位姿上添加先验以设置坐标系和第一个地标的先验来设置比例
    // 另外，由于 iSAM 以增量方式求解，我们必须等到每个都至少被观察两次将其添加到 iSAM
    if (i == 0)
    {
      // 添加一个先验的位姿 x0, 添加x,y,z标准差0.3m，roll,pitch,yaw标准差0.1rad
      noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
      graph.emplace_shared<PriorFactor<Pose3>>(Symbol('x', 0), poses[0], poseNoise);

      // 添加一个先验的路标点 l0，三维0.1m
      noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
      graph.emplace_shared<PriorFactor<Point3>>(Symbol('l', 0), points[0], pointNoise);

      // 为所有的路标点添加初始估计
      Point3 noise(-0.25, 0.20, 0.15);
      for (size_t j = 0; j < points.size(); ++j)
      {
        Point3 initial_lj = points[j] + noise;
        initialEstimate.insert(Symbol('l', j), initial_lj);
      }
    }
    else
    {
      // 使用新因子更新到 iSAM
      isam.update(graph, initialEstimate);
      // 获得优化值
      Values currentEstimate = isam.estimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      // 清除因子图和初始值，为下一次迭代存放新的数据
      graph.resize(0);
      initialEstimate.clear();
    }
  }

  return 0;
}