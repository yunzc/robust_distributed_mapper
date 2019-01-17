//
// Created by lajoiepy on 17/01/19.
//

#ifndef DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H
#define DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H

#include "distributed_mapper/DistributedMapper.h"
#include "graph_utils/graph_types.h"
#include "robot_local_map/robot_local_map.h"
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
         */
        static void solve(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                std::vector<gtsam::GraphAndValues>& graph_and_values_vector);

    };
}

#endif //DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H
