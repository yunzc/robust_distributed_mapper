// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "distributed_mapper/run_distributed_mapper.h"

using namespace std;
using namespace gtsam;

namespace distributed_mapper {

/**
 * @brief copyInitial copies the initial graph to optimized graph as a fall back option
 * @param nrRobots is the number of robots
 * @param dataDir is the directory containing the initial graph
 */
void copyInitial(size_t nrRobots, std::string dataDir) {
  cout << "Copying initial to optimized" << endl;
  for (size_t robot = 0; robot < nrRobots; robot++) {
    string dataFile_i = dataDir + boost::lexical_cast<string>(robot) + ".g2o";
    GraphAndValues graphAndValuesG2o = readG2o(dataFile_i, true);
    NonlinearFactorGraph graph = *(graphAndValuesG2o.first);
    Values initial = *(graphAndValuesG2o.second);

    // Write optimized full graph
    string distOptimized_i = dataDir + boost::lexical_cast<string>(robot) + "_optimized.g2o";
    writeG2o(graph, initial, distOptimized_i);
  }
}


/**
 * @brief function to run the whole pipeline
 */
std::tuple<double, double, int> runDistributedMapper(const size_t& nrRobots, const string& logDir, const string& dataDir, const string& traceFile, const bool& useXY, const bool& useOP,
                                                     const bool& debug, const noiseModel::Diagonal::shared_ptr& priorModel, const noiseModel::Isotropic::shared_ptr& model,
                                                     const size_t& maxIter, const double& rotationEstimateChangeThreshold, const double& poseEstimateChangeThreshold,
                                                     const double& gamma, const bool& useFlaggedInit, const distributed_mapper::DistributedMapper::UpdateType& updateType,
                                                     const bool& useBetweenNoise,  const bool& useChrLessFullGraph, const bool& useLandmarks, const double& confidence_probability, const bool& use_covariance,
                                                     const bool& usePCM) {

  vector <GraphAndValues> graphAndValuesVec; // vector of all graphs and initials

  // Config
  string robotNames_;
  if (useXY) {
    robotNames_ = string("xyz"); // robot names
  } else {
    robotNames_ = string("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"); // robot names
  }

  if (useOP) {
    robotNames_ = string("opqrstuvwxyz"); // robot names
  }

  if (useLandmarks) {
    robotNames_ = string("abcdefghijklmnopqrstyvwxyz"); // robot names
    // ABC... are used for objects
  }


  bool disconnectedGraph = false; // Flag to check whether graphs are connected or not

  ////////////////////////////////////////////////////////////////////////////////
  // Distributed Optimization
  ////////////////////////////////////////////////////////////////////////////////

  // Vector of distributed optimizers, one for each robot
  vector <boost::shared_ptr<DistributedMapper>> distMappers;

  // Load subgraph and construct distMapper optimizers
  for (size_t robot = 0; robot < nrRobots; robot++) {

    // Construct a distributed jacobi object with the given robot name
    boost::shared_ptr <DistributedMapper> distMapper(
        new DistributedMapper(robotNames_[robot], useChrLessFullGraph));

    // Read G2o files
    string dataFile_i = dataDir + boost::lexical_cast<string>(robot) + ".g2o";
    GraphAndValues graphAndValuesG2o = readG2o(dataFile_i, true);
    Values initial = *(graphAndValuesG2o.second);

    // Continue if empty
    if (initial.empty()) {
      disconnectedGraph = true;
      continue;
    }

    // Construct graphAndValues using cleaned up initial values
    GraphAndValues graphAndValues = make_pair(graphAndValuesG2o.first, boost::make_shared<Values>(initial));
    graphAndValuesVec.push_back(graphAndValues);

    // Use between noise or not in optimizePoses
    distMapper->setUseBetweenNoiseFlag(useBetweenNoise);

    // Use landmarks
    distMapper->setUseLandmarksFlag(useLandmarks);

    // Load subgraphs
    distMapper->loadSubgraphAndCreateSubgraphEdge(graphAndValues);

    // Add prior to the first robot
    if (robot == 0) {
      Key firstKey = KeyVector(initial.keys()).at(0);
      distMapper->addPrior(firstKey, initial.at<Pose3>(firstKey), priorModel);
    }

    // Verbosity level
    distMapper->setVerbosity(DistributedMapper::ERROR);

    // Check for graph connectivity
    std::set<char> neighboringRobots = distMapper->getNeighboringChars();
    if (neighboringRobots.size() == 0)
      disconnectedGraph = true;

    // Push to the set of optimizers
    distMappers.push_back(distMapper);
  }

  // Vectors containing logs
  vector <Values> rotationTrace;
  vector <Values> poseTrace;
  vector <Values> subgraphRotationTrace;
  vector <Values> subgraphPoseTrace;
  vector <VectorValues> rotationVectorValuesTrace;

  if (debug)
    cout << "Optimizing" << endl;
  // Distributed Estimate

  if (!disconnectedGraph) {
    try {
      // try optimizing
      int max_clique_size = 0;
      vector <Values> estimates = distributedOptimizer(distMappers, maxIter, max_clique_size, updateType,
                                                       gamma, rotationEstimateChangeThreshold,
                                                       poseEstimateChangeThreshold,
                                                       useFlaggedInit, useLandmarks, debug, true,
                                                       confidence_probability, use_covariance, usePCM,
                                                       graphAndValuesVec,
                                                       rotationTrace, poseTrace, subgraphRotationTrace,
                                                       subgraphPoseTrace, rotationVectorValuesTrace);

      if (debug)
        cout << "Done" << endl;

      // Aggregate estimates from all the robots
      Values distributed;
      for (size_t i = 0; i < estimates.size(); i++) {
        for (const Values::ConstKeyValuePair &key_value: estimates[i]) {
          Key key = key_value.key;
          if (!distributed.exists(key))
            distributed.insert(key, estimates[i].at<Pose3>(key));
        }

        // Write the corresponding estimate to disk
        string distOptimized_i = dataDir + boost::lexical_cast<string>(i) + "_optimized.g2o";
        writeG2o(*(graphAndValuesVec[i].first), estimates[i], distOptimized_i);
      }

      if (debug)
        cout << "Done Aggregating" << endl;

      GraphAndValues fullGraphAndValues = multirobot_util::readFullGraph(nrRobots, graphAndValuesVec);

      // Write optimized full graph
      string distOptimized = dataDir + "fullGraph_optimized.g2o";
      writeG2o(*(fullGraphAndValues.first), distributed, distOptimized);

      auto errors = multirobot_util::evaluateEstimates(nrRobots,
          fullGraphAndValues,
          priorModel,
          model,
          useBetweenNoise,
          distributed);

      return std::make_tuple(errors.first, errors.second, max_clique_size);
    }
    catch (...) {
      // Optimization failed (maybe due to disconnected graph)
      // Copy initial to optimized g2o files in that case
      copyInitial(nrRobots, dataDir);
    }
  } else {
    // Graph is disconnected
    cout << "Graph is disconnected: " << endl;
    copyInitial(nrRobots, dataDir);
  }
}
}