// In planar cases we use Pose2 variables (x, y, theta) to represent the robot poses in SE(2)
#include <gtsam/geometry/Pose2.h>

// class for factor graph, a container of various factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// class for graph nodes values, a container of various geometric types
// here Values is used as a container of SE(2)
#include <gtsam/nonlinear/Values.h>

// symbol class is used to index varible in values
// e.g. pose varibles are generally indexed as 'x' + number, and landmarks as 'l' + numbers 
#include <gtsam/inference/Symbol.h>

// Factors used in this examples
// PriorFactor gives the prior distribution over a varible
// BetweenFactor gives odometry constraints
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// optimizer class, here we use Gauss-Newton
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the 
// (appoximated / linearized) marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;

int main(int argc, char ** argv)
{
    // construct a graph
    gtsam::NonlinearFactorGraph graph;
    // define noise of 
    gtsam::noiseModel::Diagonal::shared_ptr gpsNiose = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.0));    
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.0));
    // add odometry factor to graph
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('x', 1), gtsam::Symbol('x', 2), gtsam::Pose2(1.0, 0.0, 0.0), odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('x', 2), gtsam::Symbol('x', 3), gtsam::Pose2(1.0, 0.0, 0.0), odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('x', 3), gtsam::Symbol('x', 4), gtsam::Pose2(1.0, 0.0, 0.0), odomNoise));
    // add gps factor to graph 
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('x', 1), gtsam::Pose2(0.0, 0.0, 0.0), gpsNiose));
    // graph.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('x', 2), gtsam::Pose2(1.0, 0.0, 0.0), gpsNiose));
    // graph.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('x', 3), gtsam::Pose2(2.0, 0.0, 0.0), gpsNiose));
    // graph.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('x', 4), gtsam::Pose2(3.0, 0.0, 0.0), gpsNiose));

    // add initial guess of the node
    gtsam::Values initial;
    initial.insert(gtsam::Symbol('x', 1), gtsam::Pose2(0.1, 0.1, 0.0));
    initial.insert(gtsam::Symbol('x', 2), gtsam::Pose2(1.1, 0.3, 0.0));
    initial.insert(gtsam::Symbol('x', 3), gtsam::Pose2(2.1, 0.2, 0.0));
    initial.insert(gtsam::Symbol('x', 4), gtsam::Pose2(3.1, 0.2, 0.0));

    // solve the problem
    gtsam::Values result;
    gtsam::GaussNewtonParams parameter;
    parameter.setVerbosity("ERROR");
    result = gtsam::GaussNewtonOptimizer(graph, initial, parameter).optimize();
    // print relevent information
    graph.print("\nbuild the graph\n");
    result.print("result after optimization\n");
    // get the covariance, for intuitively compare the change of uncertainty
    cout.precision(3);
    gtsam::Marginals marginals(graph, result);
    cout<< "cov of x1" << endl << marginals.marginalCovariance(gtsam::Symbol('x', 1)) << endl;
    cout<< "cov of x2" << endl << marginals.marginalCovariance(gtsam::Symbol('x', 2)) << endl;
    cout<< "cov of x3" << endl << marginals.marginalCovariance(gtsam::Symbol('x', 3)) << endl;
    cout<< "cov of x4" << endl << marginals.marginalCovariance(gtsam::Symbol('x', 4)) << endl;
    
    
    
    return 0;
};