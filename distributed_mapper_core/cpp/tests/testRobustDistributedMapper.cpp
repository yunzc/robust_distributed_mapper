/**
 * @file testRobustDistributedMapper.cpp
 * @author Pierre-Yves Lajoie (lajoie.py@gmail.com)
 * @brief unit tests for test multi robot case with outliers simulated by loading the datasets
 */
#include <distributed_mapper/runDistributedMapper.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace distributed_mapper;
using namespace multirobot_util;

/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimationWithOutliersNoRotation_2robots) {
    // Parameters
    size_t nrRobots = 2; // number of robots
    string logDir("/tmp/"); // log directory
    string dataDir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation_no_rotation/example_2robots/"); // data directory
    string traceFile("/tmp/runG2o"); // data directory
    bool useXY = false;
    bool useOP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t maxIter = 1000; // Maximum number of iterations of optimizer
    double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool useFlaggedInit = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType updateType = distributed_mapper::DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool useBetweenNoise = false; // use between factor noise or not
    bool useChrLessFullGraph = false; // whether full graph has character indexes or not
    bool useLandmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double confidence_probability = 0.99; // Confidence probability for the pairwise consistency computation
    bool useCovariance = false; // use covariance in dataset file
    bool usePCM = true; // Use pairwise consistency maximization..

    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nrRobots, logDir, dataDir, traceFile,
            useXY, useOP, debug, priorModel, model,
            maxIter, rotationEstimateChangeThreshold, poseEstimateChangeThreshold,
            gamma, useFlaggedInit, updateType, useBetweenNoise,
            useChrLessFullGraph, useLandmarks, confidence_probability, useCovariance, usePCM);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(0.0, std::get<0>(results), tolerance));
    EXPECT(assert_equal(std::get<0>(results), std::get<0>(results), tolerance));
    EXPECT(std::get<2>(results) == 10);
}

TEST(DistributedMapper, testdistributedEstimationWithoutOutliersWithRotation_2robots) {
    // Parameters
    size_t nrRobots = 2; // number of robots
    string logDir("/tmp/"); // log directory
    string dataDir("../../../test_data/pairwise_consistency_maximization/clean/simulation/example_2robots/"); // data directory
    string traceFile("/tmp/runG2o"); // data directory
    bool useXY = false;
    bool useOP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t maxIter = 1000; // Maximum number of iterations of optimizer
    double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool useFlaggedInit = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType updateType = distributed_mapper::DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool useBetweenNoise = false; // use between factor noise or not
    bool useChrLessFullGraph = false; // whether full graph has character indexes or not
    bool useLandmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double confidence_probability = 0.99; // Confidence probability for the pairwise consistency computation
    bool useCovariance = false; // use covariance in dataset file.
    bool usePCM = true; // Use pairwise consistency maximization.

    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nrRobots, logDir, dataDir, traceFile,
                                                                   useXY, useOP, debug, priorModel, model,
                                                                   maxIter, rotationEstimateChangeThreshold, poseEstimateChangeThreshold,
                                                                   gamma, useFlaggedInit, updateType, useBetweenNoise,
                                                                   useChrLessFullGraph, useLandmarks, confidence_probability, useCovariance, usePCM);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(0.0, std::get<0>(results), tolerance));
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == 10);
}

TEST(DistributedMapper, testdistributedEstimationWithOutliersWithRotation_2robots) {
    // Parameters
    size_t nrRobots = 2; // number of robots
    string logDir("/tmp/"); // log directory
    string dataDir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation/example_2robots/"); // data directory
    string traceFile("/tmp/runG2o"); // data directory
    bool useXY = false;
    bool useOP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t maxIter = 1000; // Maximum number of iterations of optimizer
    double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool useFlaggedInit = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType updateType = distributed_mapper::DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool useBetweenNoise = false; // use between factor noise or not
    bool useChrLessFullGraph = false; // whether full graph has character indexes or not
    bool useLandmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double confidence_probability = 0.99; // Confidence probability for the pairwise consistency computation
    bool useCovariance = false; // use covariance in dataset file.
    bool usePCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nrRobots, logDir, dataDir, traceFile,
                                                                   useXY, useOP, debug, priorModel, model,
                                                                   maxIter, rotationEstimateChangeThreshold, poseEstimateChangeThreshold,
                                                                   gamma, useFlaggedInit, updateType, useBetweenNoise,
                                                                   useChrLessFullGraph, useLandmarks, confidence_probability, useCovariance, usePCM);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(0.0, std::get<0>(results), tolerance));
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == 10);
}

TEST(DistributedMapper, testdistributedEstimationWithOutliersWithRotationNoPCM_2robots) {
    // Parameters
    size_t nrRobots = 2; // number of robots
    string logDir("/tmp/"); // log directory
    string dataDir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation/example_2robots/"); // data directory
    string traceFile("/tmp/runG2o"); // data directory
    bool useXY = false;
    bool useOP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t maxIter = 1000; // Maximum number of iterations of optimizer
    double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool useFlaggedInit = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType updateType = distributed_mapper::DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool useBetweenNoise = false; // use between factor noise or not
    bool useChrLessFullGraph = false; // whether full graph has character indexes or not
    bool useLandmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double confidence_probability = 0.99; // Confidence probability for the pairwise consistency computation
    bool useCovariance = false; // use covariance in dataset file.
    bool usePCM = false; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nrRobots, logDir, dataDir, traceFile,
                                                                   useXY, useOP, debug, priorModel, model,
                                                                   maxIter, rotationEstimateChangeThreshold, poseEstimateChangeThreshold,
                                                                   gamma, useFlaggedInit, updateType, useBetweenNoise,
                                                                   useChrLessFullGraph, useLandmarks, confidence_probability, useCovariance, usePCM);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(89.6, std::get<0>(results), tolerance));
    EXPECT(assert_equal(92.6, std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == 0);
}

/****************************************************************************** */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
//******************************************************************************
