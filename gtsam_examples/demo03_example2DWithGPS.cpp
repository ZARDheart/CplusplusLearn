/**
 * @file Pose2GPSExample.cpp
 * @brief 2D example with custom 'GPS' factor
 * @date Nov 8, 2016
 * @author Jing Dong
 */

/**
 * A simple 2D pose-graph SLAM with 'GPS' measurement
 * The robot moves from x1 to x3, with odometry information between each pair. 
 * each step has an associated 'GPS' measurement by GPSPose2Factor
 * The graph strcuture is shown:
 * 
 *  g1   g2   g3
 *  |    |    |
 *  x1 - x2 - x3
 */

// 自定义 GPS factor
#include "GPS2DFactor.h"

// GTSAM headers
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h> // 二元因子，位姿之间，回环之间
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // --1 新建一个因子图
  NonlinearFactorGraph graph;
  
  // --2 添加因子
  // 添加里程计因子
  // 里程计测量先验噪声模型（协方差矩阵）
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));
  // Create odometry (Between) factors between consecutive poses
  // robot makes 90 deg right turns at x3 - x5
  graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, 0), odomModel));
  
  // 添加GPS因子(不需要先验因子了)
  // 2D 'GPS' 测量先验噪声模型
  noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));
  // note that there is NO prior factor needed at first pose, since GPS provides
  // the global positions (and rotations given more than 1 GPS measurements)
  graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel));
  graph.add(GPSPose2Factor(Symbol('x', 2), Point2(5, 0), gpsModel));
  graph.add(GPSPose2Factor(Symbol('x', 3), Point2(10, 0), gpsModel));
  
  // 打印因子图
  graph.print("\nFactor Graph:\n"); 
  
  // --3 建立初始估计
  // add random noise from ground truth values
  Values initials;
  initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
  initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
  initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -0.2));
  // 打印初值
  initials.print("\nInitial Values:\n"); 


  // Use Gauss-Newton method optimizes the initial values
  GaussNewtonParams parameters;
  
  // print per iteration
  parameters.setVerbosity("ERROR");
  
  // optimize!
  GaussNewtonOptimizer optimizer(graph, initials, parameters);
  Values results = optimizer.optimize();
  
  // print final values
  results.print("Final Result:\n");

  // Calculate marginal covariances for all poses
  Marginals marginals(graph, results);
  
  // print marginal covariances
  cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;

  return 0;
}
