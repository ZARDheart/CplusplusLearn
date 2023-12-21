#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

/*
 *  p-x1 - x2 - x3
 *         |    |
 *         x5 - x4
 */

using namespace std;
using namespace gtsam;

int main(int argc, char **argv)
{
	// --1 新建一个非线性因子图
	NonlinearFactorGraph graph;

	// --2 添加因子
	// 添加先验因子
	// 假设x1上添加服从高斯分布的先验
	Pose2 priorMean(0.0, 0.0, 0.0);																		//先验均值
	noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1)); //先验标准差， 1m， 1m， 0.1弧度
	graph.add(PriorFactor<Pose2>(Symbol('x', 1), priorMean, priorModel));

	// 添加里程计因子
	noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1)); // 里程计先验噪声
	graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
	graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, -M_PI_2), odomModel));
	graph.add(BetweenFactor<Pose2>(Symbol('x', 3), Symbol('x', 4), Pose2(5, 0, -M_PI_2), odomModel));
	// 添加一个粗差
	graph.add(BetweenFactor<Pose2>(Symbol('x', 4), Symbol('x', 5), Pose2(5 + 1, 0, -M_PI_2), odomModel));

	// 添加回环因子
	noiseModel::Diagonal::shared_ptr loopModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1)); //回环检测先验噪声
	graph.add(BetweenFactor<Pose2>(Symbol('x', 5), Symbol('x', 2), Pose2(5, 0, -M_PI_2), loopModel));

	// 打印因子图
	graph.print("\nFactor Graph:\n");

	// --3 建立初始估计（不准确的）
	Values initials;
	initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
	initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
	initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -M_PI_2 - 0.2));
	initials.insert(Symbol('x', 4), Pose2(10.2, -5.0, -M_PI + 0.1));
	initials.insert(Symbol('x', 5), Pose2(5.1, -5.1, M_PI_2 - 0.1));
	// 打印初值
	initials.print("\nInitial Values:\n");

	// --4 使用g-n优化
	// Use Gauss-Newton method optimizes the initial values
	GaussNewtonParams parameters;
	// print per iteration
	parameters.setVerbosity("ERROR\n");
	// 优化
	GaussNewtonOptimizer optimizer(graph, initials, parameters);
	Values results = optimizer.optimize();
	// print values
	results.print("\nResult:\n");
	// 打印误差
	cout << "unnormalized error: " << graph.error(results) << endl;
	// 打印残差
	// graph.printErrors(results);
	// 另一种方式：
	for (int i = 0; i < graph.size(); i++)
	{
		graph.at(i)->print();
		cout << "Factor " << i << " error:  " << graph.at(i)->error(results) << endl
			 << endl;
		if ((graph.at(i)->error(results)) > 1e-3)
			// 	去掉粗差大的
			graph.remove(i);
	}

	// 重新优化
	GaussNewtonOptimizer optimizer2(graph, results, parameters);
	results = optimizer2.optimize();
	// print final values
	results.print("\nFinal Result:\n");

	// --5 Calculate marginal covariances for all poses
	// Marginals marginals(graph, results);
	// cout << "x1 covariance:\n"
	// 	 << marginals.marginalCovariance(Symbol('x', 1)) << endl;
	// cout << "x2 covariance:\n"
	// 	 << marginals.marginalCovariance(Symbol('x', 2)) << endl;
	// cout << "x3 covariance:\n"
	// 	 << marginals.marginalCovariance(Symbol('x', 3)) << endl;
	// cout << "x4 covariance:\n"
	// 	 << marginals.marginalCovariance(Symbol('x', 4)) << endl;
	// cout << "x5 covariance:\n"
	// 	 << marginals.marginalCovariance(Symbol('x', 5)) << endl;

	return 0;
}