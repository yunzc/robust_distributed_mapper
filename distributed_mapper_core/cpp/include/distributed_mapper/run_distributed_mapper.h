/**
 * @file run_distributed_mapper.h
 */
#pragma once
#include "distributed_mapper/distributed_mapper_utils.h"
#include <distributed_mapper/evaluation_utils.h>
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
     * @brief function to run the distributed mapping
     */
    std::tuple<double, double, int> runDistributedMapper(const size_t& nrRobots, const string& logDir, const string& dataDir, const string& traceFile, const bool& useXY, const bool& useOP,
                             const bool& debug, const noiseModel::Diagonal::shared_ptr& priorModel, const noiseModel::Isotropic::shared_ptr& model,
                             const size_t& maxIter, const double& rotationEstimateChangeThreshold, const double& poseEstimateChangeThreshold,
                             const double& gamma, const bool& useFlaggedInit, const distributed_mapper::DistributedMapper::UpdateType& updateType,
                             const bool& useBetweenNoise,  const bool& useChrLessFullGraph, const bool& useLandmarks, const double& confidence_probability, const bool& use_covariance,
                             const bool& usePCM);
}