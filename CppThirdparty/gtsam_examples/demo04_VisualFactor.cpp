#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <boost/make_shared.hpp>

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;

/** 创建重投影因子
 * Unary factor on the unknown pose, resulting from meauring the projection of
 * a known 3D point in the image */
class ResectioningFactor : public NoiseModelFactor1<Pose3>
{
    typedef NoiseModelFactor1<Pose3> Base;

    Cal3_S2::shared_ptr K_; ///< camera's intrinsic parameters      相机内参
    Point3 P_;              ///< 3D point on the calibration rig    世界坐标系3D点
    Point2 p_;              ///< 2D measurement of the 3D point     投影点观测值(2D)

public:
    /// 构造函数，给定 (噪声模型，待优化变量，相机内参，2D投影点，世界坐标系3D点)
    ResectioningFactor(const SharedNoiseModel &model,
                       const Key &key,
                       const Cal3_S2::shared_ptr &calib,
                       const Point2 &p,
                       const Point3 &P) : Base(model, key), K_(calib), P_(P), p_(p) {}

    /// 误差计算，形参(待优化变量，H)
    virtual Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H = boost::none) const
    {
        // 针孔相机模型 （相机位姿，内参）
        SimpleCamera camera(pose, *K_);
        // 将世界坐标系3D点，根据相机位姿（待优化变量）,投影到成像平面
        // 与观测值做差，得到重投影误差
        return camera.project(P_, H, boost::none, boost::none) - p_;
    }
};

/*******************************************************************************
 * Camera: f = 1, Image: 100x100, center: 50, 50.0
 * Pose (ground truth): (Xw, -Yw, -Zw, [0,0,2.0]')
 * Known landmarks: 已知
 *    3D Points: (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
 * Perfect measurements:
 *    2D Point:  (55,45)   (45,45)    (45,55)     (55,55)
 *******************************************************************************/
int main(int argc, char *argv[])
{
    // 相机内参
    Cal3_S2::shared_ptr calib(new Cal3_S2(1, 1, 0, 50, 50));

    /* 1. create graph */
    NonlinearFactorGraph graph;

    /* 2. add factors to the graph */
    // 噪声模型(像素误差)
    SharedDiagonal measurementNoise = Diagonal::Sigmas(Vector2(0.5, 0.5));
    // 创建上面定义的 重投影因子实例，并加入因子图
    boost::shared_ptr<ResectioningFactor> factor;
    // 注意这里的X(1) ===> 实际上使用命名空间 using symbol_shorthand::X;
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                             Point2(55, 45), Point3(10, 10, 0));
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                             Point2(45, 45), Point3(-10, 10, 0));
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                             Point2(45, 55), Point3(-10, -10, 0));
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                             Point2(55, 55), Point3(10, -10, 0));

    /* 3. Create an initial estimate for the camera pose */
    // 初始化待估计的相机位姿
    Values initial;
    initial.insert(X(1),
                   Pose3(Rot3(1.03, 0, 0, 0, -0.97, 0, 0, 0, -0.95), Point3(0, 0, 0.99)));
    // 打印因子图
    graph.print("\nFactor Graph:\n");

    /* 4. Optimize the graph using Levenberg-Marquardt*/
    // LM优化
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final result:\n");

    return 0;
}
