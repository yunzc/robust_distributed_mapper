/**
 * @file run_distributed_mapper.h
 */
#pragma once
#include "distributed_mapper/distributed_mapper_utils.h"
#include <distributed_mapper/multi_robot_utils.h>
#include <distributed_mapper/between_chordal_factor.h>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <utility>

using namespace std;
using namespace gtsam;

namespace distributed_mapper {
    /**
     * @brief readFullGraph reads the full graph if it is present in the directory, otherwise creates it
     * @param nrRobots is the number of robots
     * @param graphAndValuesVec contains the graphs and initials of each robot
     */
    GraphAndValues readFullGraph(size_t nrRobots, // number of robots
                                 vector <GraphAndValues> graphAndValuesVec  // vector of all graphs and initials for each robot
                                 );

    /**
     * @brief copyInitial copies the initial graph to optimized graph as a fall back option
     * @param nrRobots is the number of robots
     * @param dataDir is the directory containing the initial graph
     */
    void copyInitial(size_t nrRobots, std::string dataDir);


    /**
     * @brief function to run the whole pipeline
     */
    std::tuple<double, double, int> runDistributedMapper(const size_t& nrRobots, const string& logDir, const string& dataDir, const string& traceFile, const bool& useXY, const bool& useOP,
                             const bool& debug, const noiseModel::Diagonal::shared_ptr& priorModel, const noiseModel::Isotropic::shared_ptr& model,
                             const size_t& maxIter, const double& rotationEstimateChangeThreshold, const double& poseEstimateChangeThreshold,
                             const double& gamma, const bool& useFlaggedInit, const distributed_mapper::DistributedMapper::UpdateType& updateType,
                             const bool& useBetweenNoise,  const bool& useChrLessFullGraph, const bool& useLandmarks, const double& confidence_probability, const bool& use_covariance,
                             const bool& usePCM);
}