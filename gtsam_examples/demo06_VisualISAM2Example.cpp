/** @brief 模拟数据集上的结构运动问题的视觉SLAM 示例。此版本使用 iSAM2 增量解决问题 */

// For loading the data
#include "SFMdata.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>

// 我们想使用 iSAM2 逐步解决 structure-from-motion 问题，所以在此处包含 iSAM2
#include <gtsam/nonlinear/ISAM2.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <vector>
using namespace std;
using namespace gtsam;

// 基本结构和iSAM1是一致的，只需要设置一些参数
int main(int argc, char *argv[])
{
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); 
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // 创建iSAM2对象。与iSAM1不同，它执行周期性批处理步骤以保持适当的线性化和高效的变量排序，iSAM2在每一步执行部分重新线性化/重新排序。
  // 对于此示例，我们将重新线性化阈值设置得较小，因此 iSAM2 结果将接近批处理结果。
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01; //差值大于0.1需要重新线性化
  parameters.relinearizeSkip = 1;         //每当有1个值需要重新线性化时，对贝叶斯树进行更新
  /* 其他参数
  parameters.enableRelinearization=true;    //是否可以重新线性化任意变量
  parameters.evaluateNonlinearError=false;  //是否计算线性化误差默认false
  parameters.cacheLinearizedFactors = false;  //default: true 是否保存保存线性化结果，可以优化性能，但是当线性化容易计算时会造成相反效果
  parameters.factorization=ISAM2Params::Factorization::QR;  //默认为QR分解，还可以选用CHOESKY分解，但CHOESKY求解数值不稳定
  parameters.keyFormatter=DefaultKeyFormatter;  //debug时key值形式默认
  parameters.enableDetailedResults=true;  //是否计算返回ISAM2Result::detailedResults，会增加计算量
  parameters.enablePartialRelinearizationCheck=false; //是否只进行部分更新功能
  parameters.findUnusedFactorSlots=false;//当要移除许多因子时，比如ISAM2做固定平滑时，启用此选项第一值进行插入时
                                          //避免出现NULL值，但是会造成每当有新值插入时必须查找
  */
  ISAM2 isam(parameters);//贝叶斯树

  NonlinearFactorGraph graph;
  Values initialEstimate;

  for (size_t i = 0; i < poses.size(); ++i)
  {
    for (size_t j = 0; j < points.size(); ++j)
    {
      SimpleCamera camera(poses[i], *K);
      Point2 measurement = camera.project(points[j]);
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
    }
    initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));

    if (i == 0)
    {
      noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
      graph.emplace_shared<PriorFactor<Pose3>>(Symbol('x', 0), poses[0], poseNoise);

      noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
      graph.emplace_shared<PriorFactor<Point3>>(Symbol('l', 0), points[0], pointNoise); // add directly to graph

      for (size_t j = 0; j < points.size(); ++j)
        initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));
    }
    else
    {
      isam.update(graph, initialEstimate);
      // 每次调用 iSAM2 update(*) 都会执行一次迭代非线性求解器的迭代。
      // 如果以牺牲时间为代价来获得准确性，可以额外多次调用 update(*)，每一步执行多个优化器迭代。
      isam.update();
      Values currentEstimate = isam.calculateEstimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      graph.resize(0);
      initialEstimate.clear();
    }
  }

  return 0;
}