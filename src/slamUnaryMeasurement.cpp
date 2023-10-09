#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include "include/unaryFactor.h"


int main(int argc, char ** argv)
{
    gtsam::NonlinearFactorGraph graph;
    
    // set up prior
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1.0, 1.0, 0.01 * M_PI));
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(gtsam::Symbol('X', 0), gtsam::Pose2(0.1, 0.1, 0), priorNoise));
    // set up odom
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.01 * M_PI));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', 0), gtsam::Symbol('X', 1), gtsam::Pose2(1.0, 0.0, 0), odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', 1), gtsam::Symbol('X', 2), gtsam::Pose2(1.0, 0.0, 0), odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', 2), gtsam::Symbol('X', 3), gtsam::Pose2(0.0, -1.0, -0.5 * M_PI), odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', 3), gtsam::Symbol('X', 4), gtsam::Pose2(-1.0, 0.0, -0.5 * M_PI), odomNoise));
    // set up loop closure
    gtsam::noiseModel::Diagonal::shared_ptr loopNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.01 * M_PI));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(gtsam::Symbol('X', 4), gtsam::Symbol('X', 1), gtsam::Pose2(1.0, 1.0, -M_PI), loopNoise));
    // set up unary measurement
    gtsam::noiseModel::Diagonal::shared_ptr gpsNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.01 * M_PI));
    graph.add(GpsFactor(gtsam::Symbol('X', 1), gtsam::Pose2(0.9, 1.1, 0), gpsNoise));
    graph.add(GpsFactor(gtsam::Symbol('X', 2), gtsam::Pose2(1.1, 1.0, 0), gpsNoise));
    graph.add(GpsFactor(gtsam::Symbol('X', 3), gtsam::Pose2(1.1, 1.0, 0), gpsNoise));
    graph.add(GpsFactor(gtsam::Symbol('X', 4), gtsam::Pose2(0.9, 0.9, 0), gpsNoise));
    
    // set the initial values of the nodes
    gtsam::Values initials;
    initials.insert(gtsam::Symbol('X', 0), gtsam::Pose2(0.0, 0.0, 0));
    initials.insert(gtsam::Symbol('X', 1), gtsam::Pose2(1.0, 0.0, 0));
    initials.insert(gtsam::Symbol('X', 2), gtsam::Pose2(2.0, 0.0, 0));
    initials.insert(gtsam::Symbol('X', 3), gtsam::Pose2(2.0, -1.0, -0.5 * M_PI));
    initials.insert(gtsam::Symbol('X', 4), gtsam::Pose2(1.0, -1.0, -M_PI));

    // set uo solver
    gtsam::GaussNewtonParams parameters;
    parameters.setVerbosity("errors");

    // solve
    gtsam::Values results = gtsam::GaussNewtonOptimizer(graph, initials, parameters).optimize();

    // print optimization information 
    graph.print("\nbuild the graph\n");
    results.print("result after optimization\n");
    std::cout.precision(3);
    gtsam::Marginals marginals(graph, results);
    std::cout << "x1 cov: " << std::endl << marginals.marginalCovariance(gtsam::Symbol('X', 1)) << std::endl;
    std::cout << "x2 cov: " << std::endl << marginals.marginalCovariance(gtsam::Symbol('X', 2)) << std::endl;
    std::cout << "x3 cov: " << std::endl << marginals.marginalCovariance(gtsam::Symbol('X', 3)) << std::endl;
    std::cout << "x4 cov: " << std::endl << marginals.marginalCovariance(gtsam::Symbol('X', 4)) << std::endl;
    
    

    return 0;
}