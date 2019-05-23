// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "global_map/global_map.h"
#include <math.h>

#ifdef LOG_DIR
#define DIR LOG_DIR
#else
#define DIR "../../"
#endif

namespace global_map {

const std::string GlobalMap::LOG_DIRECTORY = std::string(DIR) + std::string("/log/");
const std::string GlobalMap::CONSISTENCY_MATRIX_FILE_NAME = std::string(GlobalMap::LOG_DIRECTORY+"consistency_matrix.clq.mtx");
const std::string GlobalMap::CONSISTENCY_LOOP_CLOSURES_FILE_NAME = std::string(GlobalMap::LOG_DIRECTORY+"consistent_loop_closures.txt");

GlobalMap::GlobalMap(const robot_measurements::RobotLocalMap& robot1_local_map,
                const robot_measurements::RobotLocalMap& robot2_local_map,
                const robot_measurements::RobotMeasurements& interrobot_measurements,
                const double& confidence_probability):
                pairwise_consistency_(robot1_local_map.getTransforms(), robot2_local_map.getTransforms(), 
                            interrobot_measurements.getTransforms(), interrobot_measurements.getLoopClosures(),
                            robot1_local_map.getTrajectory(), robot2_local_map.getTrajectory(),
                            robot1_local_map.getNbDegreeFreedom(), confidence_probability){}


std::vector<int> GlobalMap::pairwiseConsistencyMaximization() {
    // Compute consistency matrix
    Eigen::MatrixXi consistency_matrix = pairwise_consistency_.computeConsistentMeasurementsMatrix();
    graph_utils::printConsistencyGraph(consistency_matrix, CONSISTENCY_MATRIX_FILE_NAME);
    
    // Compute maximum clique
    FMC::CGraphIO gio;
    gio.readGraph(CONSISTENCY_MATRIX_FILE_NAME);
    int max_clique_size = 0;
    std::vector<int> max_clique_data;
    max_clique_size = FMC::maxClique(gio, max_clique_size, max_clique_data);

    // Print results
    graph_utils::printConsistentLoopClosures(pairwise_consistency_.getLoopClosures(), max_clique_data, CONSISTENCY_LOOP_CLOSURES_FILE_NAME);

    return max_clique_data;
}

}