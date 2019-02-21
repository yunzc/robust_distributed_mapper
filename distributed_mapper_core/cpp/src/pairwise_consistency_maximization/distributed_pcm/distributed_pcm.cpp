// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "pairwise_consistency_maximization/distributed_pcm/distributed_pcm.h"

namespace distributed_pcm {

    int DistributedPCM::solve(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                                const double& confidence_probability, const bool& use_covariance){

        std::vector<graph_utils::LoopClosures> separators_by_robot;
        std::vector<graph_utils::Transforms> transforms_by_robot;
        std::map<std::pair<char, char>,graph_utils::Transforms> separators_transforms_by_pair;
        // Intialization of the pairs
        for (int i = 0; i < dist_mappers.size(); i++) {
            for (int j = i+1; j < dist_mappers.size(); j++) {
                graph_utils::Transforms transforms;
                separators_transforms_by_pair.insert(std::make_pair(std::make_pair(dist_mappers[i]->robotName(),dist_mappers[j]->robotName()), transforms));
            }
        }
        for (const auto& dist_mapper : dist_mappers) {
            // Store separators key pairs
            graph_utils::LoopClosures separators;
            for (auto id : dist_mapper->seperatorEdge()) {
                auto separator_edge = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                        dist_mapper->currentGraph().at(id));
                separators.emplace_back(std::make_pair(separator_edge->key1(), separator_edge->key2()));
            }
            separators_by_robot.emplace_back(separators);

            graph_utils::Transforms transforms;
            bool id_initialized = false;
            for (const auto& factor_ptr : dist_mapper->currentGraph()) {
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
                        separators_transforms_by_pair.at(std::make_pair(gtsam::symbolChr(edge_ptr->key1()),gtsam::symbolChr(edge_ptr->key2()))).transforms.insert(
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
        int total_max_clique_sizes = 0;
        for (int roboti = 0; roboti < dist_mappers.size(); roboti++) {
            for (int robotj = roboti+1; robotj < dist_mappers.size(); robotj++) {

                auto roboti_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[roboti], separators_by_robot[roboti]);
                auto robotj_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[robotj], separators_by_robot[robotj]);
                auto roboti_robotj_separators_transforms = separators_transforms_by_pair.at(std::make_pair(dist_mappers[roboti]->robotName(),dist_mappers[robotj]->robotName()));
                auto interrobot_measurements = robot_measurements::InterRobotMeasurements(roboti_robotj_separators_transforms, dist_mappers[roboti]->robotName(), dist_mappers[robotj]->robotName());

                auto global_map = global_map::GlobalMap(roboti_local_map, robotj_local_map, interrobot_measurements, confidence_probability);
                std::vector<int> max_clique = global_map.pairwiseConsistencyMaximization();

                // Retrieve indexes of rejected measurements
                auto robot_pair = {roboti, robotj};
                for (auto robot : robot_pair) {
                    auto separators_ids = dist_mappers[robot]->seperatorEdge();
                    int number_separators = separators_ids.size();
                    std::cout << "Robot " << robot << " : Number of separators : "  << number_separators << std::endl;
                    std::vector<int> rejected_separator_ids;
                    for (int i = 0; i < separators_ids.size(); i++) {
                        if (std::find(max_clique.begin(), max_clique.end(), i) == max_clique.end()) { // TODO: Add a comparison function to reject the right measurments
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
                        dist_mappers[robot]->eraseFactor(id);
                        graph_and_values_vector.at(robot).first->erase(graph_and_values_vector.at(robot).first->begin()+id);
                    }
                    std::vector<size_t> new_separator_ids;
                    for (size_t i = separators_ids[0]; i < separators_ids[0]+number_separators; i++) {
                        new_separator_ids.emplace_back(i);
                    }
                    dist_mappers[robot]->setSeparatorIds(new_separator_ids);
                }
                total_max_clique_sizes += max_clique.size();
            }
        }

        return total_max_clique_sizes;
    }

}