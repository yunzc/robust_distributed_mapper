// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "pairwise_consistency_maximization/distributed_pcm/distributed_pcm.h"

namespace distributed_pcm {

    int DistributedPCM::solve(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                                const double& confidence_probability, const bool& use_covariance){

        std::vector<graph_utils::LoopClosures> separators_by_robot;
        std::vector<graph_utils::Transforms> transforms_by_robot;
        graph_utils::Transforms separators_transforms;
        for (const auto& distMapper : dist_mappers) {
            // Store separators key pairs
            graph_utils::LoopClosures separators;
            for (auto id : distMapper->seperatorEdge()) {
                auto separator_edge = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                        distMapper->currentGraph().at(id));
                separators.emplace_back(std::make_pair(separator_edge->key1(), separator_edge->key2()));
            }
            separators_by_robot.emplace_back(separators);

            graph_utils::Transforms transforms;
            bool id_initialized = false;
            for (const auto& factor_ptr : distMapper->currentGraph()) {
                auto edge_ptr = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor_ptr);
                if (edge_ptr) { // Possible bug : the graph size is 17, however there are only 16 edges..
                    graph_utils::Transform transform;
                    transform.i = edge_ptr->key1();
                    transform.j = edge_ptr->key2();
                    transform.pose.pose = edge_ptr->measured();
                    if (use_covariance) {
                        transform.pose.covariance_matrix =
                                boost::dynamic_pointer_cast< gtsam::noiseModel::Gaussian >(edge_ptr->noiseModel())->covariance();
                    } else {
                        transform.pose.covariance_matrix = graph_utils::FIXED_COVARIANCE;
                    }
                    transform.is_separator = std::find(separators.begin(), separators.end(),
                                                       std::make_pair(edge_ptr->key1(), edge_ptr->key2())) !=
                                                       separators.end();
                    if (!transform.is_separator) {
                        if (!id_initialized) {
                            transforms.start_id = transform.i;
                            transforms.end_id = transform.j;
                            id_initialized = true;
                        } else {
                            transforms.start_id = std::min(transforms.start_id, transform.i);
                            transforms.end_id = std::max(transforms.end_id, transform.j);
                        }
                        transforms.transforms.insert(
                                std::make_pair(std::make_pair(edge_ptr->key1(), edge_ptr->key2()), transform));
                    } else {
                        separators_transforms.transforms.insert(
                                std::make_pair(std::make_pair(edge_ptr->key1(), edge_ptr->key2()), transform));
                    }
                }
            }
            transforms_by_robot.emplace_back(transforms);
        }

        // Apply PCM for each robot
        // TODO: Consider N robots case.
        // TODO: Communication of the factors needed for optimization
        // For now I will work with perfect information

        auto robot1_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[0], separators_by_robot[0]);
        auto robot2_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[1], separators_by_robot[1]);
        auto interrobot_measurements = robot_measurements::RobotMeasurements(separators_transforms, separators_by_robot[0]);

        auto global_map = global_map::GlobalMap(robot1_local_map, robot2_local_map, interrobot_measurements, confidence_probability);
        std::vector<int> max_clique = global_map.pairwiseConsistencyMaximization();

        // Retrieve indexes of rejected measurements
        for (int robot = 0; robot < dist_mappers.size(); robot++) {//distMapper : dist_mappers
            auto separators_ids = dist_mappers[robot]->seperatorEdge();
            int number_separators = separators_ids.size();
            std::cout << "Robot " << robot << " : Number of separators : "  << number_separators << std::endl;
            std::vector<int> rejected_separator_ids;
            for (int i = 0; i < separators_ids.size(); i++) {
                if (std::find(max_clique.begin(), max_clique.end(), i) == max_clique.end()) {
                    rejected_separator_ids.emplace_back(i);
                    number_separators--;
                }
            }
            // Remove measurements not in the max clique
            // TODO: Fix "off by one" bug in innerEdges_ and graph_
            std::cout << "Robot " << robot << " : Size of the maximal consistency clique : "  << max_clique.size() << std::endl;
            int number_separator_ids_removed = 0;
            for (const auto& index : rejected_separator_ids) {
                auto id = separators_ids[index] - number_separator_ids_removed;
                number_separator_ids_removed++;
                auto pose = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                        dist_mappers[robot]->currentGraph().at(id))->measured();
                dist_mappers[robot]->eraseFactor(id);
                graph_and_values_vector.at(robot).first->erase(graph_and_values_vector.at(robot).first->begin()+id);
            }
            std::vector<size_t> new_separator_ids;
            for (size_t i = separators_ids[0]; i < separators_ids[0]+number_separators; i++) {
                new_separator_ids.emplace_back(i);
            }
            dist_mappers[robot]->setSeparatorIds(new_separator_ids);
        }
        return max_clique.size();
    }

}