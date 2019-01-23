#pragma once 

#include "distributed_pcm/distributed_pcm.h"
#include <fstream>
#include <algorithm>
#include <map>
#include <utility>


namespace distributed_mapper{

/**
 * @brief orderRobots orders the robots using flagged init
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param nrRobots is the number of robots
 * @param robotNames is the name of robots
 * @param useFlaggedInit orders using flagged initialization
 * @return
 */
std::vector<size_t>
orderRobots(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
            size_t nrRobots,
            std::string robotNames,
            bool useFlaggedInit,
            bool useLandmarks = false);

/**
 * @brief logRotationTrace
 * @param distMapper_robot
 * @param distributed_vectorvalues_iter
 * @param distributed_iter
 * @param subgraph_iter
 */
std::pair<gtsam::Values, gtsam::VectorValues> logRotationTrace(boost::shared_ptr<DistributedMapper> distMapper_robot);


/**
 * @brief optimizeRotation optimizes the rotation in a distributed sense
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param maxIter is the maximum number of iteratinos
 * @param nrRobots is the number of robots
 * @param robotNames is the name of robots
 * @param ordering is the prioritized ordering
 * @param rotationEstimateChangeThreshold provides an early stopping condition
 * @param rotationTrace contains the overall converted estimate at any iteration
 * @param subgraphRotationTrace contains the Values for a particular subgraph being optimized
 * @param rotationVectorValuesTrace contains the linearized rotation estimate
 */
void optimizeRotation(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
                      size_t maxIter,
                      size_t nrRobots,
                      std::string robotNames,
                      std::vector<size_t> ordering,
                      bool debug,
                      double rotationEstimateChangeThreshold,
                      bool useLandmarks = false,
                      boost::optional<std::vector<gtsam::Values>&> rotationTrace = boost::none,
                      boost::optional<std::vector<gtsam::Values>&> subgraphRotationTrace = boost::none,
                      boost::optional<std::vector<gtsam::VectorValues>&> rotationVectorValuesTrace = boost::none,
                      boost::optional<gtsam::VectorValues&> rotationCentralized = boost::none,
                      boost::optional<gtsam::GaussianFactorGraph&> rotationGraph = boost::none);


/**
 * @brief optimizePose optimizes the pose in a distributed sense
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param maxIter is the maximum number of iterations
 * @param nrRobots is the number of robots
 * @param robotNames is the name of robots
 * @param ordering is the prioritized ordering
 * @param poseEstimateChangeThreshold provides an early stopping condition
 */
void optimizePose(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
                  size_t maxIter,
                  size_t nrRobots,
                  std::string robotNames,
                  std::vector<size_t> ordering,
                  bool debug,
                  double poseEstimateChangeThreshold,
                  bool useLandmarks = false,
                  boost::optional<std::vector<gtsam::Values>&> poseTrace = boost::none,
                  boost::optional<std::vector<gtsam::Values>&> subgraphPoseTrace = boost::none,
                  boost::optional<gtsam::Values&> poseCentralized = boost::none,
                  boost::optional<gtsam::NonlinearFactorGraph&> poseGraph = boost::none);

/**
 * @brief distributedOptimization  performs distributed estimation using Jacobi algorithm
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param maxIter is the maximum number of iteration
 * @param rotationTrace if exists, it will store the trace of rotation estimates after each iteration
 * @param poseTrace if exists, it will store the trace of pose estimates after each iteration
 * @return the estimated vector of poses for each robot
 */
std::vector< gtsam::Values >
distributedOptimizer(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
                     size_t maxIter,
                     int& max_clique_size,
                     DistributedMapper::UpdateType updateType = DistributedMapper::incUpdate,
                     double gamma = 1.0f,
                     double rotationEstimateChangeThreshold = 1e-5,
                     double poseEstimateChangeThreshold = 1e-6,
                     bool useFlaggedInit = false,
                     bool useLandmarks = false,
                     bool debug = false,
                     bool contains_odometry = true,
                     const double& confidence_probability = 0.99,
                     const bool& use_covariance = false,
                     const bool& use_pcm = true,
                     boost::optional<std::vector<gtsam::GraphAndValues>&> graphAndValuesVec = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> rotationTrace = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> poseTrace  = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> subgraphRotationTrace = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> subgraphPoseTrace  = boost::none,
                     boost::optional<std::vector<gtsam::VectorValues>&> rotationVectorValuesTrace  = boost::none,
                     boost::optional<gtsam::VectorValues&> rotationCentralized = boost::none,
                     boost::optional<gtsam::Values&> poseCentralized = boost::none,
                     boost::optional<gtsam::NonlinearFactorGraph&> graphWithoutPrior = boost::none,
                     boost::optional<gtsam::GaussianFactorGraph&> centralizedRotationGraph = boost::none);


////////////////////////////////////////////////////////////////////////////////
// Logging
////////////////////////////////////////////////////////////////////////////////
void logResults(size_t nrRobots,
                std::string traceFile,
                gtsam::Values centralized,
                std::vector< boost::shared_ptr<DistributedMapper> > distMappers);

}
