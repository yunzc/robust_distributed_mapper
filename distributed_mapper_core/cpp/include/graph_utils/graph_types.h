// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GRAPH_UTILS_TYPES_H
#define GRAPH_UTILS_TYPES_H

#include <gtsam/base/Value.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <map>
#include <vector>

/** \namespace graph_utils
 *  \brief This namespace encapsulates utility functions to manipulate graphs
 */
namespace graph_utils {

/** \struct Transform
 *  \brief Structure defining a transformation between two poses
 */
struct Transform {
    gtsam::Key i, j;
    gtsam::Pose3 pose;
    bool is_loop_closure;
};

/** \struct Transforms
 *  \brief Structure defining a std::map of transformations
 */
struct Transforms {
    gtsam::Key start_id, end_id;
    std::map<std::pair<gtsam::Key,gtsam::Key>, graph_utils::Transform> transforms;
};

/** \struct TrajectoryPose
 *  \brief Structure defining a pose in a robot trajectory
 */
struct TrajectoryPose {
    gtsam::Key id;
    gtsam::Pose3 pose;
};

/** \struct Trajectory
 *  \brief Structure defining a robot trajectory
 */
struct Trajectory {
    gtsam::Key start_id, end_id;
    std::map<gtsam::Key, graph_utils::TrajectoryPose> trajectory_poses;
};

/** \typedef LoopClosures
 *  \brief type to store poses IDs of loop closures.
 */
/** Type defining a list of pair of poses with a loop closure */
typedef std::vector<std::pair<gtsam::Key,gtsam::Key>> LoopClosures;

}
#endif