// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "pairwise_consistency_maximization/distributed_pcm/distributed_pcm.h"

namespace distributed_pcm {

    int DistributedPCM::solve(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                                const double& confidence_probability){

        std::vector<graph_utils::LoopClosures> separatorsByRobot;
        std::vector<graph_utils::Transforms> transformsByRobot;
        graph_utils::Transforms separatorsTransforms;
        for (auto distMapper : dist_mappers) {
            // What information do I have access?

            // auto separatorEdge = (*separatorEdges[0]); // gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
            // auto measurement = separatorEdge.measured(); // measurements between the nodes : gtsam::LieGroup<gtsam::Pose3, 6>
            // auto rotation = separatorEdge.measured().R(); // rotation part : gtsam::LieGroup<gtsam::Rot3, 3>
            // auto translation = separatorEdge.measured().t(); // transaltion part : Eigen::Matrix<double, 3, 1, 0, 3, 1>
            // auto node1 = separatorEdge.key1(); // Key : uint64_t
            // auto node2 = separatorEdge.key2(); // Key : uint64_t

            // Store separators key pairs
            graph_utils::LoopClosures separators;
            for (auto id : distMapper->seperatorEdge()) {
                auto separatorEdge = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                        distMapper->currentGraph().at(id));
                separators.emplace_back(std::make_pair(separatorEdge->key1(), separatorEdge->key2()));
            }
            separatorsByRobot.emplace_back(separators);
            // Place all measurement edges in a map and store the keys of separators
            // std::cout << "currentGraph size: " << distMapper->currentGraph().size() << '\n';

            graph_utils::Transforms transforms;
            bool idInitialized = false;
            for (auto factorPtr : distMapper->currentGraph()) {
                auto edgePtr = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factorPtr);
                if (edgePtr) { // Possible bug : the graph size is 17, however there are only 16 edges..
                    graph_utils::Transform transform;
                    transform.i = edgePtr->key1();
                    transform.j = edgePtr->key2();
                    transform.pose.pose = edgePtr->measured();
                    // TODO: Read covariance from file or add option
                    transform.pose.covariance_matrix = graph_utils::FIXED_COVARIANCE;
                    transform.is_separator = std::find(separators.begin(), separators.end(),
                                                       std::make_pair(edgePtr->key1(), edgePtr->key2())) !=
                                             separators.end();
                    if (!transform.is_separator) {
                        if (!idInitialized) {
                            transforms.start_id = transform.i;
                            transforms.end_id = transform.j;
                            idInitialized = true;
                        } else {
                            transforms.start_id = std::min(transforms.start_id, transform.i);
                            transforms.end_id = std::max(transforms.end_id, transform.j);
                        }
                        transforms.transforms.insert(
                                std::make_pair(std::make_pair(edgePtr->key1(), edgePtr->key2()), transform));
                    } else {
                        separatorsTransforms.transforms.insert(
                                std::make_pair(std::make_pair(edgePtr->key1(), edgePtr->key2()), transform));
                    }
                }
            }
            transformsByRobot.emplace_back(transforms);
        }

        // Apply PCM for each robot
        // TODO: Consider N robots case.
        // TODO: Communication of the factors needed for optimization
        // For now I will work with perfect information

        auto robot1LocalMap = robot_local_map::RobotLocalMap(transformsByRobot[0], separatorsByRobot[0]);
        auto robot2LocalMap = robot_local_map::RobotLocalMap(transformsByRobot[1], separatorsByRobot[1]);
        auto interrobotMeasurements = robot_local_map::RobotMeasurements(separatorsTransforms, separatorsByRobot[0]);

        auto globalMap = global_map::GlobalMap(robot1LocalMap, robot2LocalMap, interrobotMeasurements, confidence_probability);
        std::vector<int> max_clique = globalMap.pairwiseConsistencyMaximization();

        // Retrieve indexes of rejected measurements
        for (int robot = 0; robot < dist_mappers.size(); robot++) {//distMapper : dist_mappers
            auto separatorsIds = dist_mappers[robot]->seperatorEdge();
            int numberSeparators = separatorsIds.size();
            std::cout << "Robot " << robot << " : Number of separators : "  << numberSeparators << std::endl;
            std::vector<int> rejectedSeparatorIds;
            for (int i = 0; i < separatorsIds.size(); i++) {
                if (std::find(max_clique.begin(), max_clique.end(), i) == max_clique.end()) {
                    rejectedSeparatorIds.emplace_back(i);
                    numberSeparators--;
                }
            }
            // Remove measurements not in the max clique
            // TODO: Fix "off by one" bug in innerEdges_ and graph_
            std::cout << "Robot " << robot << " : Size of the maximal consistency clique : "  << max_clique.size() << std::endl;
            int numberSeparatorIdsRemoved = 0;
            for (auto index : rejectedSeparatorIds) {
                auto id = separatorsIds[index] - numberSeparatorIdsRemoved;
                numberSeparatorIdsRemoved++;
                auto pose = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                        dist_mappers[robot]->currentGraph().at(id))->measured();
                dist_mappers[robot]->eraseFactor(id);
                graph_and_values_vector.at(robot).first->erase(graph_and_values_vector.at(robot).first->begin()+id);
            }
            std::vector<size_t> newSeparatorIds;
            for (size_t i = separatorsIds[0]; i < separatorsIds[0]+numberSeparators; i++) {
                newSeparatorIds.emplace_back(i);
            }
            dist_mappers[robot]->setSeparatorIds(newSeparatorIds);
        }
        return max_clique.size();
    }

}