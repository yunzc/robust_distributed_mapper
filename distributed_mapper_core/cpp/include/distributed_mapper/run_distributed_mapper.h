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
                                 ) {
        std::cout << "Creating fullGraph by combining subgraphs." << std::endl;

        // Combined graph and Values
        NonlinearFactorGraph::shared_ptr combinedGraph(new NonlinearFactorGraph);
        Values::shared_ptr combinedValues(new Values);

        // Iterate over each robot
        for (size_t robot = 0; robot < nrRobots; robot++) {

            // Load graph and initial
            NonlinearFactorGraph graph = *(graphAndValuesVec[robot].first);
            Values initial = *(graphAndValuesVec[robot].second);

            // Iterate over initial and push it to the combinedValues, each initial value is present only once
            for (const Values::ConstKeyValuePair &key_value: initial) {
                Key key = key_value.key;
                if (!combinedValues->exists(key))
                    combinedValues->insert(key, initial.at<Pose3>(key));
            }

            // Iterate over the graph and push the factor if it is not already present in combinedGraph
            for (size_t ksub = 0; ksub < graph.size(); ksub++) { //for each factor in the new subgraph
                bool isPresent = false;
                for (size_t k = 0; k < combinedGraph->size(); k++) {

                    boost::shared_ptr <BetweenFactor<Pose3>> factorSub =
                            boost::dynamic_pointer_cast < BetweenFactor < Pose3 > > (graph.at(ksub));
                    Key factorSubKey1 = factorSub->key1();
                    Key factorSubKey2 = factorSub->key2();

                    boost::shared_ptr <BetweenFactor<Pose3>> factorCombined =
                            boost::dynamic_pointer_cast < BetweenFactor < Pose3 > > (combinedGraph->at(k));
                    Key factorCombinedKey1 = factorCombined->key1();
                    Key factorCombinedKey2 = factorCombined->key2();

                    // values don't match exactly that's why check with keys as well
                    if (factorCombined->equals(*factorSub) ||
                        ((factorSubKey1 == factorCombinedKey1) && (factorSubKey2 == factorCombinedKey2))) {
                        isPresent = true;
                        break;
                    }
                }
                if (isPresent == false) // we insert the factor
                    combinedGraph->add(graph.at(ksub));
            }
        }

        // Return graph and values
        return make_pair(combinedGraph, combinedValues);
    }

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

                    // Write the corresponding estimate in TUM format
                    string distOptimized_i_tum = dataDir + boost::lexical_cast<string>(i) + "_optimizedTUM.txt";
                    multirobot_util::writeValuesAsTUM(estimates[i], distOptimized_i_tum);
                }

                if (debug)
                    cout << "Done Aggregating" << endl;


                ////////////////////////////////////////////////////////////////////////////////
                // Read full graph and add prior
                ////////////////////////////////////////////////////////////////////////////////
                GraphAndValues fullGraphAndValues = readFullGraph(nrRobots, graphAndValuesVec);
                NonlinearFactorGraph fullGraph = *(fullGraphAndValues.first);
                Values fullInitial = *(fullGraphAndValues.second);

                // Add prior
                NonlinearFactorGraph fullGraphWithPrior = fullGraph.clone();
                Key priorKey = KeyVector(fullInitial.keys()).at(0);
                NonlinearFactor::shared_ptr prior(
                        new PriorFactor<Pose3>(priorKey, fullInitial.at<Pose3>(priorKey), priorModel));
                fullGraphWithPrior.push_back(prior);

                // Write optimized full graph
                string distOptimized = dataDir + "fullGraph_optimized.g2o";
                writeG2o(fullGraph, distributed, distOptimized);

                ////////////////////////////////////////////////////////////////////////////////
                // Chordal Graph
                ////////////////////////////////////////////////////////////////////////////////
                NonlinearFactorGraph chordalGraph = distributed_mapper::multirobot_util::convertToChordalGraph(
                        fullGraph, model, useBetweenNoise);

                ////////////////////////////////////////////////////////////////////////////////
                // Initial Error
                ////////////////////////////////////////////////////////////////////////////////
                std::cout << "Initial Error: " << chordalGraph.error(fullInitial) << std::endl;

                ////////////////////////////////////////////////////////////////////////////////
                // Centralized Two Stage
                ////////////////////////////////////////////////////////////////////////////////
                Values centralized = distributed_mapper::multirobot_util::centralizedEstimation(fullGraphWithPrior,
                                                                                                model, priorModel,
                                                                                                useBetweenNoise);
                double centralized_error = chordalGraph.error(centralized);
                std::cout << "Centralized Two Stage Error: " << centralized_error << std::endl;

                ////////////////////////////////////////////////////////////////////////////////
                // Centralized Two Stage + Gauss Newton
                ////////////////////////////////////////////////////////////////////////////////
                Values chordalGN = distributed_mapper::multirobot_util::centralizedGNEstimation(fullGraphWithPrior,
                                                                                                model, priorModel,
                                                                                                useBetweenNoise);
                std::cout << "Centralized Two Stage + GN Error: " << chordalGraph.error(chordalGN) << std::endl;

                ////////////////////////////////////////////////////////////////////////////////
                // Distributed Error
                ////////////////////////////////////////////////////////////////////////////////
                double distributed_error = chordalGraph.error(distributed);
                std::cout << "Distributed Error: " << distributed_error << std::endl;

                return std::make_tuple(centralized_error, distributed_error, max_clique_size);
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