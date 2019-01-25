// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H
#define DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H

#include "distributed_mapper/distributed_mapper.h"
#include "graph_utils/graph_types.h"
#include "robot_measurements/robot_local_map.h"
#include "global_map/global_map.h"

/** \namespace distributed_pcm
 *  \brief This namespace encapsulates the pairwise consistency maximization in a distributed setup.
 */
namespace distributed_pcm {
    /** \class DistributedPCM
     * \brief Class computing the pairwise consistency maximization with partial information.
     */
    class DistributedPCM {
        public:
        /** \brief Constructor
         */
        DistributedPCM(){};

        /**
         * \brief Function that solves the global maps according to the current constraints
         *
         * @param dist_mappers is the different distributed mappers in the system (one by robot)
         * @param graph_and_values is the collection of factors of all graph used for evaluation
         * @returns size of the maximum clique of pairwise consistent measurements
         */
        static int solve(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                const double& confidence_probability, const bool& use_covariance);

    };
}

#endif //DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H
