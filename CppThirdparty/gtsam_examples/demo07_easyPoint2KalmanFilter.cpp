/**
 * @file easyPoint2KalmanFilter.cpp
 *
 * simple linear Kalman filter on a moving 2D point, but done using factor graphs
 * This example uses the templated ExtendedKalmanFilter class to perform the same
 * operations as in elaboratePoint2KalmanFilter
 *
 * @date Aug 19, 2011
 * @author Frank Dellaert
 * @author Stephen Williams
 */
#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>

using namespace std;
using namespace gtsam;

// Define Types for Linear System Test
typedef Point2 LinearMeasurement;

int main()
{
  // --1 创建初始点
  // Create the Kalman Filter initialization point
  Point2 x_initial(0.0, 0.0);
  SharedDiagonal P_initial = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));
  // Create Key for initial pose
  Symbol x0('x', 0);
  // Create an ExtendedKalmanFilter object
  ExtendedKalmanFilter<Point2> ekf(x0, x_initial, P_initial);

  // --2 预测
  // 现在预测 t=1 时的状态，即 argmax_{x1} P(x1) = P(x1|x0) P(x0)
  // 在卡尔曼滤波器表示法中，这是 x_{t+1|t} 和 P_{t+1|t}
  // 对于卡尔曼滤波器，这需要一个运动模型，f(x_{t}) = x_{t+1|t)
  // 假设系统是线性的，形式为 x_{t+1} = f(x_{t}) = F*x_{t} + B*u_{t} + w
  // 其中 F 是状态转换模型/矩阵，B 是控制输入模型，并且 w 是零均值的高斯白噪声，协方差为 Q
  // 注意，在某些模型中，Q 实际上导出为 G*w*G^T，其中 w 模拟某些不确定性物理属性，如速度或加速度，G来源于物理
  //
  // 出于本示例的目的，假设我们使用的是恒定位置模型，并且控件以 1 m/s 的速度向右驱动该点。 
  // 那么，F = [1 0 ; 0 1], B = [1 0 ; 0 1] 和 u = [1 ; 0]。 我们还假设过程噪声 Q = [0.1 0 ; 0 0.1]。
  Vector u = Vector2(1.0, 0.0);
  SharedDiagonal Q = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1), true);
  // 这个简单的运动可以用BetweenFactor建模
  // 为下一个位姿创建 Key
  Symbol x1('x', 1);
  // 基于控件预测增量(视为里程计)
  Point2 difference(u);
  // Create Factor
  BetweenFactor<Point2> factor1(x0, x1, difference, Q);
  // Predict the new value with the EKF class
  Point2 x1_predict = ekf.predict(factor1);
  traits<Point2>::Print(x1_predict, "X1 Predict");

  // --3 更新
  // 现在，已经接收到测量值 z1，并且卡尔曼滤波器应该是“更新”/“更正”，这相当于说 P(x1|z1) ~ P(z1|x1)*P(x1)
  // 对于卡尔曼滤波器，这需要一个测量模型 h(x_{t}) = \hat{z}_{t}
  // 假设系统是线性的，其形式为 z_{t} = h(x_{t}) = H*x_{t} + v
  // 其中 H 是观察模型/矩阵，v 是零均值的高斯白噪声，协方差为 R
  //
  // 为了这个例子的目的，让我们假设我们有一个类似 GPS 的东西，它返回机器人的当前位置。 
  // 那么 H = [1 0 ; 0 1]。 我们还假设测量噪声R = [0.25 0 ; 0 0.25]。
  SharedDiagonal R = noiseModel::Diagonal::Sigmas(Vector2(0.25, 0.25), true);
  // This simple measurement can be modeled with a PriorFactor
  Point2 z1(1.1, 0.0);
  PriorFactor<Point2> factor2(x1, z1, R);
  // Update the Kalman Filter with the measurement
  Point2 x1_update = ekf.update(factor2);
  traits<Point2>::Print(x1_update, "X1 Update");
  ekf.print();

  // Do the same thing two more times...
  // Predict
  Symbol x2('x', 2);
  difference = Point2(1, 0);
  BetweenFactor<Point2> factor3(x1, x2, difference, Q);
  Point2 x2_predict = ekf.predict(factor1);
  traits<Point2>::Print(x2_predict, "X2 Predict");

  // Update
  Point2 z2(1.8, 0.2);
  PriorFactor<Point2> factor4(x2, z2, R);
  Point2 x2_update = ekf.update(factor4);
  traits<Point2>::Print(x2_update, "X2 Update");

  // Do the same thing one more time...
  // Predict
  Symbol x3('x', 3);
  difference = Point2(1, 0);
  BetweenFactor<Point2> factor5(x2, x3, difference, Q);
  Point2 x3_predict = ekf.predict(factor5);
  traits<Point2>::Print(x3_predict, "X3 Predict");

  // Update
  Point2 z3(3.1, -0.1);
  PriorFactor<Point2> factor6(x3, z3, R);
  Point2 x3_update = ekf.update(factor6);
  traits<Point2>::Print(x3_update, "X3 Update");

  return 0;
}