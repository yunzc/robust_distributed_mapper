// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "pairwise_consistency_maximization/distributed_pcm/distributed_pcm.h"

namespace distributed_pcm {

    int DistributedPCM::solveCentralized(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
            std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
            const double& confidence_probability, const bool& use_covariance) {

        std::vector<graph_utils::LoopClosures> separators_by_robot;
        std::vector<graph_utils::Transforms> transforms_by_robot;
        std::map<std::pair<char, char>,graph_utils::Transforms> separators_transforms_by_pair;

        fillInRequiredInformationCentralized(separators_by_robot, transforms_by_robot, separators_transforms_by_pair,
                dist_mappers, use_covariance);

        // Apply PCM for each pair of robots
        int total_max_clique_sizes = 0;
        for (int roboti = 0; roboti < dist_mappers.size(); roboti++) {
            for (int robotj = roboti+1; robotj < dist_mappers.size(); robotj++) {

                int max_clique_size = executePCMCentralized(roboti, robotj, transforms_by_robot, separators_by_robot,
                separators_transforms_by_pair, dist_mappers,
                graph_and_values_vector, confidence_probability);

                total_max_clique_sizes += max_clique_size;
            }
        }

        return total_max_clique_sizes;
    }

    int DistributedPCM::solveDecentralized(const int& other_robot_id,
                boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper,
                gtsam::GraphAndValues& local_graph_and_values,
                const gtsam::Values& other_robot_poses,
                const double& confidence_probability, const bool& use_covariance) {

        graph_utils::LoopClosures separators;
        graph_utils::Transforms transforms;
        graph_utils::Trajectory other_robot_trajectory;
        graph_utils::Transforms separators_transforms;

        fillInRequiredInformationDecentralized(separators, transforms, other_robot_trajectory, separators_transforms,
                dist_mapper, other_robot_poses, other_robot_id, use_covariance);

        // Apply PCM for each pair of robots
        int max_clique_size = executePCMDecentralized(other_robot_id, transforms,
                                                    separators, other_robot_trajectory,
                                                    separators_transforms, dist_mapper,
                                                    local_graph_and_values, confidence_probability);

        return max_clique_size;
    }

    void DistributedPCM::fillInRequiredInformationCentralized(std::vector<graph_utils::LoopClosures>& separators_by_robot,
                        std::vector<graph_utils::Transforms>& transforms_by_robot,
                        std::map<std::pair<char, char>,graph_utils::Transforms>& separators_transforms_by_pair,
                        const std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                        const bool& use_covariance){
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
            for (auto id : dist_mapper->separatorEdge()) {
                auto separator_edge = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                        dist_mapper->currentGraph().at(id));
                separators.emplace_back(std::make_pair(separator_edge->key1(), separator_edge->key2()));
            }
            separators_by_robot.emplace_back(separators);

            graph_utils::Transforms transforms;
            bool id_initialized = false;
            for (const auto& factor_ptr : dist_mapper->currentGraph()) {
                auto edge_ptr = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor_ptr);
                if (edge_ptr) { // Do nothing with the prior in the first robot graph
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
    }

    void DistributedPCM::fillInRequiredInformationDecentralized(graph_utils::LoopClosures& separators,
                        graph_utils::Transforms& transforms,
                        graph_utils::Trajectory& other_robot_trajectory,
                        graph_utils::Transforms& separators_transforms,
                        const boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper,
                        const gtsam::Values& other_robot_poses,
                        const int& other_robot_id,
                        const bool& use_covariance){
        // Store separators key pairs
        for (auto id : dist_mapper->separatorEdge()) {
            auto separator_edge = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                    dist_mapper->currentGraph().at(id));
            separators.emplace_back(std::make_pair(separator_edge->key1(), separator_edge->key2()));
        }

        bool id_initialized = false;
        for (const auto& factor_ptr : dist_mapper->currentGraph()) {
            auto edge_ptr = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor_ptr);
            if (edge_ptr) { // Do nothing with the prior in the first robot graph
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
                    auto id_1 = gtsam::Symbol(edge_ptr->key1()).chr()-97;
                    auto id_2 = gtsam::Symbol(edge_ptr->key2()).chr()-97;
                    if (id_1 == other_robot_id || id_2 == other_robot_id) {
                        separators_transforms.transforms.insert(
                                std::make_pair(std::make_pair(edge_ptr->key1(), edge_ptr->key2()), transform));
                    }
                }
            }
        }

        // Fill in the other robot poses in a trajectory structure
        gtsam::Key first_key, last_key;
        bool init_first = false;
        bool init_last = false;
        for (const gtsam::Values::ConstKeyValuePair& key_value_pair : other_robot_poses) {  
            if (first_key > key_value_pair.key || !init_first) {
                first_key = key_value_pair.key;
                init_first = true;
            }
            if (last_key < key_value_pair.key || !init_last) {
                last_key = key_value_pair.key;
                init_last = true;
            }

            graph_utils::TrajectoryPose trajectory_pose;
            trajectory_pose.id = key_value_pair.key;
            trajectory_pose.pose.pose = other_robot_poses.at<gtsam::Pose3>(key_value_pair.key);
            other_robot_trajectory.trajectory_poses.insert(std::make_pair(key_value_pair.key, trajectory_pose));
        }
        other_robot_trajectory.start_id = first_key;
        other_robot_trajectory.end_id = last_key;
    }

    int DistributedPCM::executePCMCentralized(const int& roboti, const int& robotj, const std::vector<graph_utils::Transforms>& transforms_by_robot,
                const std::vector<graph_utils::LoopClosures>& separators_by_robot,
                const std::map<std::pair<char, char>,graph_utils::Transforms>& separators_transforms_by_pair,
                std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                const double& confidence_probability){
        auto roboti_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[roboti], separators_by_robot[roboti]);
        auto robotj_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[robotj], separators_by_robot[robotj]);
        auto roboti_robotj_separators_transforms = separators_transforms_by_pair.at(std::make_pair(dist_mappers[roboti]->robotName(),dist_mappers[robotj]->robotName()));
        auto interrobot_measurements = robot_measurements::InterRobotMeasurements(roboti_robotj_separators_transforms, dist_mappers[roboti]->robotName(), dist_mappers[robotj]->robotName());

        auto global_map = global_map::GlobalMap(roboti_local_map, robotj_local_map, interrobot_measurements, confidence_probability);
        std::vector<int> max_clique = global_map.pairwiseConsistencyMaximization();

        // Retrieve indexes of rejected measurements
        auto robot_pair = {roboti, robotj};
        for (auto robot : robot_pair) {
            auto separators_ids = dist_mappers[robot]->separatorEdge();
            std::vector<int> rejected_separator_ids;
            for (int i = 0; i < separators_ids.size(); i++) {
                if (isSeparatorToBeRejected(max_clique, separators_ids[i], roboti_robotj_separators_transforms,
                                            interrobot_measurements.getLoopClosures(), dist_mappers[robot])) {
                    rejected_separator_ids.emplace_back(i);
                }
            }
            // Remove measurements not in the max clique
            int number_separator_ids_removed = 0;
            for (const auto& index : rejected_separator_ids) {
                auto id = separators_ids[index] - number_separator_ids_removed;
                number_separator_ids_removed++;
                dist_mappers[robot]->eraseFactor(id);
                graph_and_values_vector.at(robot).first->erase(graph_and_values_vector.at(robot).first->begin()+id);
            }
            // Update separator ids
            std::vector<size_t> new_separator_ids;
            int number_of_edges = dist_mappers[robot]->currentGraph().size();
            if (robot == 0){
                // Do not count the prior in the first robot graph
                number_of_edges--;
            }
            for (int i = 0; i < number_of_edges; i++) {
                auto keys = dist_mappers[robot]->currentGraph().at(i)->keys();
                char robot0 = gtsam::symbolChr(keys.at(0));
                char robot1 = gtsam::symbolChr(keys.at(1));
                if (robot0 != robot1) {
                    new_separator_ids.push_back(i);
                }
            }
            dist_mappers[robot]->setSeparatorIds(new_separator_ids);
        }
        return max_clique.size();
    }

    int DistributedPCM::executePCMDecentralized(const int& other_robot_id, const graph_utils::Transforms& transforms,
                                            const graph_utils::LoopClosures& separators,
                                            const graph_utils::Trajectory& other_robot_trajectory,
                                            const graph_utils::Transforms& separators_transforms,
                                            boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper,
                                            gtsam::GraphAndValues& local_graph_and_values,
                                            const double& confidence_probability){

        auto robot_local_map = robot_measurements::RobotLocalMap(transforms, separators);
        graph_utils::Transforms empty_transforms;
        auto other_robot_local_info = robot_measurements::RobotLocalMap(other_robot_trajectory, empty_transforms, separators);
        auto roboti_robotj_separators_transforms = separators_transforms;
        auto interrobot_measurements = robot_measurements::InterRobotMeasurements(roboti_robotj_separators_transforms, 
                                                                                dist_mapper->robotName(), ((char) other_robot_id + 97));

        auto global_map = global_map::GlobalMap(robot_local_map, other_robot_local_info, interrobot_measurements, confidence_probability);
        std::vector<int> max_clique = global_map.pairwiseConsistencyMaximization();

        // Retrieve indexes of rejected measurements
        auto separators_ids = dist_mapper->separatorEdge();
        std::vector<int> rejected_separator_ids;
        for (int i = 0; i < separators_ids.size(); i++) {
            if (isSeparatorToBeRejected(max_clique, separators_ids[i], roboti_robotj_separators_transforms,
                                        interrobot_measurements.getLoopClosures(), dist_mapper)) {
                rejected_separator_ids.emplace_back(i);
            }
        }
        // Remove measurements not in the max clique
        int number_separator_ids_removed = 0;
        for (const auto& index : rejected_separator_ids) {
            auto id = separators_ids[index] - number_separator_ids_removed;
            number_separator_ids_removed++;
            dist_mapper->eraseFactor(id);
            local_graph_and_values.first->erase(local_graph_and_values.first->begin()+id);
        }
        // Update separator ids
        std::vector<size_t> new_separator_ids;
        int number_of_edges = dist_mapper->currentGraph().size();
        if (((int) dist_mapper->robotName()-97) == 0){
            // Do not count the prior in the first robot graph
            number_of_edges--;
        }
        for (int i = 0; i < number_of_edges; i++) {
            auto keys = dist_mapper->currentGraph().at(i)->keys();
            char robot0 = gtsam::symbolChr(keys.at(0));
            char robot1 = gtsam::symbolChr(keys.at(1));
            if (robot0 != robot1) {
                new_separator_ids.push_back(i);
            }
        }
        dist_mapper->setSeparatorIds(new_separator_ids);

        return max_clique.size();
    }

    bool DistributedPCM::isSeparatorToBeRejected(const std::vector<int>& max_clique, const int& separtor_id, const graph_utils::Transforms& separators_transforms,
                                        const graph_utils::LoopClosures& loop_closures, boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper) {

        auto separator_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(dist_mapper->currentGraph().at(separtor_id));
        // First check if the separator is between the 2 robots
        if (!((gtsam::symbolChr(separator_factor->keys().at(0)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.first)
                && gtsam::symbolChr(separator_factor->keys().at(1)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.second)) ||
              (gtsam::symbolChr(separator_factor->keys().at(0)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.second)
                && gtsam::symbolChr(separator_factor->keys().at(1)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.first)))){
            return false;
        }
        // Check if in the maximum clique
        auto key_pair = std::make_pair(separator_factor->keys().at(0), separator_factor->keys().at(1));
        int index;
        for (index = 0; index < loop_closures.size(); index++) {
            if (key_pair == loop_closures[index]) {
                break;
            }
        }
        if (std::find(max_clique.begin(), max_clique.end(), index) != max_clique.end()) {
            return false;
        }
        return true;
    }

}