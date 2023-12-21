#include "SFMdata.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <vector>
using namespace std;
using namespace gtsam;

int main(int argc, char *argv[])
{
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); 
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  NonlinearFactorGraph graph;
  Values initialEstimate;
  
  initialEstimate.insert(Symbol('x', 0), poses[0]);
  
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); 
  graph.emplace_shared<PriorFactor<Pose3>>(Symbol('x', 0), poses[0], poseNoise);
      
      isam.update(graph, initialEstimate);
      Values currentEstimate = isam.calculateEstimate();
      cout << "****************************************************" << endl;
      graph.print();
      currentEstimate.print("Current estimate: ");

  return 0;
}
